// Probing and setup based on uart_bus_fdt.c

#include <sys/types.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bitset.h>
#include <sys/_bitset.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/memdesc.h>
#include <sys/mutex.h>

#include <machine/bus.h>
#include <machine/bus_dma.h>
#include <machine/bus_dma_impl.h>
#include <machine/resource.h>

#include <dev/iocap/iocap.h>
#include <dev/iocap/iocap_keymngr.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "dev/fdt/simplebus.h"

// WE ARE ASSUMING WE ARE COMPILING FOR RISC-V HERE. WATCH OUT WHEN COMPILING FOR SOMETHING ELSE
#define TOKENPASTE(a, b) a ## b // "##" is the "Token Pasting Operator"
#define TOKENPASTE2(a,b) TOKENPASTE(a, b) // expand then paste
#define static_assert(x, msg) enum { TOKENPASTE2(ASSERT_line_,__LINE__) \
= 1 / (msg && (x)) }

// static_assert(offsetof(struct bus_dma_tag, common) == 0, "struct bus_dma_tag must start with a bus_dma_tag_common field");
static_assert(offsetof(struct bus_dma_tag_common, impl) == 0,
		"struct bus_dma_tag_common must start with an impl vtable");

static MALLOC_DEFINE(M_IOCAP_DMATAG, "iocap_dmatag", "IOCAP DMA Tag");
static MALLOC_DEFINE(M_IOCAP_DMAMAP, "iocap_dmamap", "IOCAP DMA Map");
static MALLOC_DEFINE(M_IOCAP_DMAMAP_QUARANTINE, "iocap_dmamap_quarantine", "IOCAP DMA Map Quarantine Entry");

#define MAX_NUM_KEYS_PER_TAG 4

struct bus_dma_iocap_refinable_tag {
	struct bus_dma_tag_common common;
	device_t iocap_keymngr;
	// Unfortunate reality: we need to malloc() a pointer to (struct bus_dma_tag) by getting a DMA tag from our parent,
	// we can't redirect calls here.
	bus_dma_tag_t base_tag;
	// TODO IS THIS NECESSARY
	// // Set to true if there are any non-iocap mappings created through this tag.
	// // If so, we don't refine it(?)
	// int has_non_iocap_mappings;
};

// Previously, I kept a permanent 256-entry array of key states inside the key manager,
// which required taking a lock on the key manager whenever you use the keys.
// In an effort to make the key manager functions more scalable to multi-core systems, a few changes have been made.
// 1. Instead of allocating a full buffer of all key data for all keys ever created,
// store individual key statuses inside DMA tag objects as needed.
// This means accessing keys does not require taking a lock on the key manager.
// 2. Instead of allocating a full buffer of all key data for all keys, use a bitmap to tell
// if keys are allocated to DMA tags or not.
// This allows cheap lookup, especially if you have CLZ instructions.
struct iocap_key_state {
	CCapU128 key_data __aligned(8);
	uint8_t key_id;
	// Does the key currently have usable contents i.e. can it be used to mint iocaps
	bool active;
};

struct iocap_keymngr_quarantined_mem_cb {
	SLIST_ENTRY(iocap_keymngr_quarantined_mem_cb) entries;
	iocap_keymngr_bus_dmamap_unload2_cb cb;
	void* cb_arg1;
	void* cb_arg2;
};

SLIST_HEAD(iocap_keymngr_quarantine_queue, iocap_keymngr_quarantined_mem_cb);

struct iocap_key_suballocator {
	struct mtx mtx;
	struct iocap_key_state key_datas[MAX_NUM_KEYS_PER_TAG];
	uint8_t n_keys;
	struct iocap_keymngr_revocation_params params;
	union {
		struct {
			// List of callbacks to call once the key is revoked
			struct iocap_keymngr_quarantine_queue quarantine;
			// Increases when a new mapping is mapped, decreased when a mapping is unmapped
			uint64_t active_mappings;
			// Monotonic increases, unmapping does not decrease
			uint64_t total_mappings;
		} single_refcounted_key;
		struct {
			// Per-epoch statistics
			struct {
				// List of callbacks to call once the epoch key is revoked
				struct iocap_keymngr_quarantine_queue quarantine;
				// Increases when a new mapping is mapped, decreased when a mapping is unmapped.
				// When nonzero, the mapping is active
				uint64_t active_mappings;
				// Monotonic decreases, unmapping does not increase
				uint64_t num_mappings_left;
				// // Monotonic decreases, unmapping does not increase
				// uint64_t bytes_mapped_left;
			} key_stats[MAX_NUM_KEYS_PER_TAG];

			// The insertion_key may indicate an entry in key_data where key_data is inactive.
			// Even if insertion_key points to a valid epoch,
			// the key_stats[insertion_key_plus_one] may be full, and is_full = true.
			// When is_full = true, no insertions can take place.
			//	The first revoked key will replace insertion_key_plus_one and set is_full = false.
			uint8_t insertion_key;
			bool is_full;
		} rolling_epochs_x4;
	};
};

struct bus_dma_iocap_enabled_tag {
	struct bus_dma_tag_common common;
	bus_dma_tag_t base_tag;
	device_t iocap_keymngr;

	struct iocap_key_suballocator key_suballoc;
};

//                               IOCapblt
#define BUS_IOCAP_DMAMAP_MAGIC 0x10CA9B17u

// DMA maps have a simple lifecycle.
// When they are initially created, they have no actual mapping loaded into them - creation simply
// allocates all the memory they may need to actually create a mapping up front.
// From a consumer's perspective, one call to a bus_dmamap_load_{...}() function fills the DMA map with a
// mapping of a resource (depending on the bus_dmamap_load variant) to a list of bus-addressable DMA segments.
// The DMA map does not(?) store the array of segments itself, so you can't access the output segments after loading.
// Consumers are expected to keep track of those segments and their association with the map.
// You cannot(?) call bus_dmamap_load_{...}() on the same dmamap twice - you have to bus_dmamap_unload() first.

// DMA maps have a set of functions that can be invoked on them, that are stored in their parent tag's vtable.
// .tag_create = create a derived instance of a tag from a parent tag, requires malloc-ing the tag
// .tag_destroy = destroy a tag, requires free-ing the tag

// from sys/bus_dma.h

/*
* Allocate a handle for mapping from kva/uva/physical
* address space into bus device space.
*/
// .map_create = from a tag, create a DMA map object which does not have any memory regions loaded. requires malloc-ing the map
/*
 * Destroy a handle for mapping from kva/uva/physical
 * address space into bus device space.
 */
// .map_destroy = destroy a DMA map object. requires free-ing the map
/*
 * Allocate a piece of memory that can be efficiently mapped into
 * bus device space based on the constraints listed in the dma tag.
 * A dmamap to for use with dmamap_load is also allocated.
 */
// .mem_alloc = from a tag, simultaneously create a DMA map and allocate a memory region which can be (but isn't initially) loaded into the dmamap
/*
 * Free a piece of memory and its allocated dmamap, that was allocated
 * via bus_dmamem_alloc.
 */
// .mem_free = simultaneously free a DMA map object and memory region created through .mem_alloc
/*
 * Release the mapping held by map.
 */
// .map_unload = refinable_map_unload,
/*
 * Perform a synchronization operation on the given map. If the map
 * is NULL we have a fully IO-coherent system.
 */
// .map_sync = ensure that if any DMAs happened to the segments associated with this DMA map, they are synchronized with the actual memory we mapped into those segements.

// Other functions in sys/bus_dma.h, most notably the bus_dmamap_load_{mbuf,bio,etc.}() variants,
// are defined in sys/kern/subr_bus_dma.c in terms of these other functions from the vtable which are not exposed directly.
// They are not explicitly documented anywhere(?) and thus these descriptions are inferred from usage.
// .load_phys = load a physically-addressed buffer into the mapping
// .load_buffer = load a virtually-addressed buffer into the mapping
// .load_ma = load an array of virtual memory pages into the mapping
// .map_waitok = setup common fields in all dma maps (the struct memdesc describing the memory being mapped in, the callback field and argument, and the parent tag.
// .map_complete = return either the passed in bus_dma_segment_t* or your own array of allocated bus_dma_segment_t* if the former is NULL (??)

// This means in practice a DMA map is in one of three states:
// - unloaded, after an unload() or create()
// - loading, after any load_{phys,buffer,ma}()
// - loaded, after map_waitok() and map_complete() have been called.
// You cannot go directly from unloaded to loaded, you must go through loading (i.e. at least one region must be loaded in).
// This is relevant to IOCaps in managing key lifetimes. When does a DMA map select a key ID from its parent tag?
// When does it decrement the reference count of that key ID?
// My initial guess was that we increment the reference count of the key ID once loaded, and decrement it only on unload().
// However, there's a crucial problem. the load_{...}() functions are specified as non-blocking.
// On top of that, some drivers like virtio_blk may load_{...}() then do more work, detect and error, and immediately unload()
// without sending it to the device!
// So instead we can select the key ID on bus_dmamap_sync, which is also required before enqueueing, which also
// carries an implicit expectation of being a bit slow (e.g. copying to bounce buffers).
// TODO THIS DEPENDS ON EVERYONE CALLING SYNC BEFORE ENQUEUING - IS THAT REALISTIC?
// To that end, introduce an extra state
// - key_selected, after bus_dmamap_sync()
enum bus_iocap_dmamap_state {
	iocap_dmamap_unloaded,
	iocap_dmamap_loading,
	iocap_dmamap_loaded,
	iocap_dmamap_key_selected,
};

struct bus_iocap_dmamap {
	// A magic number that identifies this as an IOCap-capable mapping
	uint32_t magic;
	enum bus_iocap_dmamap_state state;
	bus_dmamap_t base_map;
	struct bus_dma_iocap_enabled_tag *tag;
	int nth_key_of_tag;
};


extern struct bus_dma_impl bus_dma_iocap_refinable_tag_impl;

static int iocap_keymngr_probe(device_t);

static int iocap_keymngr_attach(device_t);

static int iocap_keymngr_detach(device_t);

static bus_get_dma_tag_t iocap_keymngr_get_dma_tag;

BITSET_DEFINE(iocap_keymngr_avail_keys, 256);

struct iocap_keymngr_softc {
	struct simplebus_softc base;

	device_t dev;

	// see sys/dev/uart/uart.h
	bus_space_tag_t bst;
	bus_space_handle_t bsh;

	struct resource *sc_rres; /* Register resource. */
	int sc_rrid;
	int sc_rtype; /* SYS_RES_{IOPORT|MEMORY}. */

	struct mtx iocap_keymngr_mtx;

	struct bus_dma_iocap_refinable_tag base_refinable_tag;

	// Bitmask for allocated keys, 256-bits for now, but expandable if needed.
	// Bit #N is 1 if key N is free, 0 otherwise.
	struct iocap_keymngr_avail_keys available_keys;
};

static device_method_t iocap_keymngr_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, iocap_keymngr_probe),
	DEVMETHOD(device_attach, iocap_keymngr_attach),
	DEVMETHOD(device_detach, iocap_keymngr_detach),

	DEVMETHOD(bus_get_dma_tag, iocap_keymngr_get_dma_tag),

	{ 0, 0 }
};

DEFINE_CLASS_1(iocap_keymngr, iocap_keymngr_driver, iocap_keymngr_methods,
		sizeof(struct iocap_keymngr_softc), simplebus_driver);

EARLY_DRIVER_MODULE(iocap_keymngr, ofwbus, iocap_keymngr_driver, 0, 0,
		BUS_PASS_BUS);
EARLY_DRIVER_MODULE(iocap_keymngr, simplebus, iocap_keymngr_driver, 0, 0,
		BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE);


// device_printf the value of all performance counters
static void iocap_keymngr_dbg_perfcounters(device_t);
// sysctl-compatible function for reading performance counters through MMIO.
// Takes a iocap_keymngr_softc* in arg1, and an address to u64-read in arg2.
// SYSCTL_ADD_PROC is employed to ensure that arg2 is always 0x1000, 0x1008, 0x1010, or 0x1018.
// IT WOULD BE BAD IF SOMEONE PASSED A DIFFERENT VALUE IN. IS THAT POSSIBLE?
static int iocap_keymngr_dbg_perfcounters_sysctl(SYSCTL_HANDLER_ARGS);

// Assign n_key_ids key IDs from the key manager to a tag, without reusing key IDs already assigned to other tags.
//
// Takes the mutex on the key manager.
static int iocap_keymngr_alloc_key_ids(device_t, struct iocap_key_suballocator *key_suballoc);

// Fill the key data for this id with random data so it can be used to mint iocaps.
//
// Assumes the relevant key allocator mutex is held.
static void iocap_keymngr_init_key(device_t, struct iocap_key_state *key_state);

// Clear out the key data for this ID.
//
// Assumes the relevant key allocator mutex is held.
static void iocap_keymngr_clear_key(device_t, struct iocap_key_state *key_state);

// Clear data for all given key IDs in a key suballocator and mark them as not-allocated in the device so other tags can reuse them.
// TODO need some way to ensure this doesn't leave a key open and usable,
//
// Assumes the relevant key suballocator mutex is held.
// Takes the key manager mutex.
static void iocap_keymngr_free_key_ids(device_t, struct iocap_key_suballocator* key_suballoc);


// Assign a key within a tag for a map.
// Increment the refcount for that key on the given tag.
// If the refcount increases from zero for that key call iocap_keymngr_init_key().
//
// Takes the key allocator mutex for the tag.
static int
iocap_enabled_tag_assign_key(bus_dma_iocap_enabled_tag_t tag,
		bus_iocap_dmamap_t map);

// Decrement the refcount for a map's key on a given tag and transitions the map to iocap_dmamap_unloaded.
// If the map is in iocap_dmamap_loaded, does nothing.
// If the map is otherwise not in iocap_dmamap_key_selected, asserts.
//
// When the refcount hits zero, call iocap_keymngr_clear_key() to clear the key data, and call all quarantine callbacks, but keep the key index allocated.
//
// If cb is not NULL and the refcount hits zero, calls cb(cb_arg1, cb_arg2) after the other quarantine callbacks.
// If cb is not NULL and the refcount doesn't hit 0, enqueues (cb, cb_arg1, cb_arg2) to the relevant quarantine queue.
//
// Takes the key allocator mutex for the tag if the map is in iocap_dmamap_key_selected.
static int
iocap_enabled_tag_unassign_key(bus_dma_iocap_enabled_tag_t tag,
	bus_iocap_dmamap_t map, iocap_keymngr_bus_dmamap_unload2_cb cb,
	void* cb_arg1, void* cb_arg2);


static int
iocap_keymngr_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_is_compatible(dev, "sws35,iocap_keymngr")) {
		device_set_desc(dev, "IOCap Key Manager");
		return (BUS_PROBE_DEFAULT);
	}

	return (ENXIO);
}

static int
iocap_keymngr_attach(device_t dev)
{
	device_printf(dev, "w00t attached to iocap!!! parent: %p\n",
			device_get_parent(dev));

	struct iocap_keymngr_softc *sc;
	// phandle_t node;
	int error;

	sc = device_get_softc(dev);
	// node = ofw_bus_get_node(dev);
	error = 0;

	// Step one: get the register block allocated for the key manager.

	// From uart_core.c:uart_bus_probe
	/*
	 * Allocate the register resource. We assume that all UARTs have
	 * a single register window in either I/O port space or memory
	 * mapped I/O space. Any UART that needs multiple windows will
	 * consequently not be supported by this driver as-is. We try I/O
	 * port space first because that's the common case.
	 */
	// See also goldfish_rtc.c

	// Get the resource
	sc->sc_rrid = 0;
	sc->sc_rtype = SYS_RES_MEMORY;
	sc->sc_rres = bus_alloc_resource_any(dev, sc->sc_rtype, &sc->sc_rrid,
			RF_ACTIVE);
	if (sc->sc_rres == NULL) {
		device_printf(dev, "could not allocate resource\n");
		error = ENXIO;
		goto fail;
	}
	// Get the mapped memory for the resource
	sc->bsh = rman_get_bushandle(sc->sc_rres);
	sc->bst = rman_get_bustag(sc->sc_rres);

	sc->base_refinable_tag.common.impl = &bus_dma_iocap_refinable_tag_impl;
	sc->base_refinable_tag.iocap_keymngr = dev;
	error = bus_dma_tag_create(
		bus_get_dma_tag(dev),
		1, // any alignment
		0, // no boundary restrictions
		BUS_SPACE_MAXADDR, // excluded zone = 0 (BUS_SPACE_MAXADDR..BUS_SPACE_MAXADDR)
		BUS_SPACE_MAXADDR,
		NULL, NULL, // filtfunc and filtfunc arg are deprecated
		maxphys,					/* max request size */
		16,	/* TODO max # segments is this enough */
		maxphys,					/* max segment size */
		0, // no flags
		NULL, NULL, // lockfunc, lockfuncarg not used
		&sc->base_refinable_tag.base_tag
	);
	if (error != 0) {
		device_printf(dev, "could not allocate dma tag from parent %s\n", device_get_name(device_get_parent(dev)));
		goto fail;
	}

	/* Initialize available_keys to filled (all bits set) */
	sc->available_keys = (struct iocap_keymngr_avail_keys) BITSET_T_INITIALIZER(BITSET_FSET(__bitset_words(256)));

	// Add the sysctls
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *tree;
	struct sysctl_oid_list *child;

	// Not sure what this is - got it from virtio_blk.c
	ctx = device_get_sysctl_ctx(dev);
	tree = device_get_sysctl_tree(dev);
	child = SYSCTL_CHILDREN(tree);

	// Add a function-based sysctl called good_read
	// whireh returns a U64, is read-only, and multiprocess-safe(?)
	// which calls iocap_keymngr_dbg_perfcounters_sysctl(sc, 0x1000)
	// which returns a U64 (or a 'QU').
	SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "good_write",
	    CTLTYPE_U64 | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0x1000,
	    iocap_keymngr_dbg_perfcounters_sysctl, "QU",
	    "Number of correct IOCap writes handled by the key manager");
	// Ditto for the rest
	SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "bad_write",
	    CTLTYPE_U64 | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0x1008,
	    iocap_keymngr_dbg_perfcounters_sysctl, "QU",
	    "Number of incorrect IOCap writes handled by the key manager");
	SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "good_read",
	    CTLTYPE_U64 | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0x1010,
	    iocap_keymngr_dbg_perfcounters_sysctl, "QU",
	    "Number of correct IOCap reads handled by the key manager");
	SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "bad_read",
	    CTLTYPE_U64 | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0x1018,
	    iocap_keymngr_dbg_perfcounters_sysctl, "QU",
	    "Number of incorrect IOCap reads handled by the key manager");

	iocap_keymngr_dbg_perfcounters(dev);

	// Setup lock
	// TODO: Using MTX_DEF right now because I assume there are no points where an iocap_keymngr locked function needs to be reentrant. Is that... sensible?
	// It's what virtio_blk does, but that might not be built to handle multithreading
	mtx_init(&sc->iocap_keymngr_mtx, "IOCap Key Manager Lock", NULL, MTX_DEF);

fail:
	if (error) {
		iocap_keymngr_detach(dev);
		return (error);
	}

	return simplebus_attach(dev);
}

static int
iocap_keymngr_detach(device_t dev)
{
	int err = simplebus_detach(dev);
	if (err != 0)
		return err;

	struct iocap_keymngr_softc *sc;

	sc = device_get_softc(dev);

	{
		// Lock key manager
		mtx_lock(&sc->iocap_keymngr_mtx);

		if (sc->base_refinable_tag.base_tag != NULL) {
			bus_dma_tag_destroy(sc->base_refinable_tag.base_tag);
			sc->base_refinable_tag.base_tag = NULL;
		}

		if (sc->sc_rres != NULL) {
			bus_release_resource(dev, sc->sc_rtype, sc->sc_rrid, sc->sc_rres);
			sc->sc_rres = NULL;
		}

		// Unlock key manager
		mtx_unlock(&sc->iocap_keymngr_mtx);
	}

	mtx_destroy(&sc->iocap_keymngr_mtx);

	return err;
}

static void
iocap_keymngr_dbg_perfcounters(device_t dev)
{
	struct iocap_keymngr_softc *sc;

	sc = device_get_softc(dev);

	// device_printf(dev, "iocap_keymngr_dbg_perfcounters bst %p bsh %zu bsz %zu\n", sc->bst, (size_t)sc->bsh, (size_t)sc->bsz);
	uint64_t good_read = bus_space_read_8(sc->bst, sc->bsh, 0x1000);
	// device_printf(dev, "iocap_keymngr_dbg_perfcounters good_read\n");
	uint64_t bad_read = bus_space_read_8(sc->bst, sc->bsh, 0x1008);
	// device_printf(dev, "iocap_keymngr_dbg_perfcounters bad_read\n");
	uint64_t good_write = bus_space_read_8(sc->bst, sc->bsh, 0x1010);
	// device_printf(dev, "iocap_keymngr_dbg_perfcounters good_write\n");
	uint64_t bad_write = bus_space_read_8(sc->bst, sc->bsh, 0x1018);
	device_printf(dev, "perf counters: %ld %ld %ld %ld\n", good_read,
			bad_read, good_write, bad_write);
}

static int
iocap_keymngr_dbg_perfcounters_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct iocap_keymngr_softc *sc;
	bus_size_t address;
	uint64_t value;
	int error;

	sc = arg1;
	address = arg2;
	value = bus_space_read_8(sc->bst, sc->bsh, address);
	error = sysctl_handle_64(oidp, &value, 0, req);

	return error;
}

// Assign n_key_ids key IDs to a tag, without reusing key IDs already assigned to other tags
static int iocap_keymngr_alloc_key_ids(device_t dev, struct iocap_key_suballocator *key_suballoc)
{
	struct iocap_keymngr_softc *sc;

	sc = device_get_softc(dev);

	// Lock key manager
	mtx_lock(&sc->iocap_keymngr_mtx);

	if (BIT_COUNT(256, &sc->available_keys) < key_suballoc->n_keys) {
		// Unlock key manager
		mtx_unlock(&sc->iocap_keymngr_mtx);
		device_printf(dev, "Ran out of keys\n");

		return ENOSPC;
	}

	device_printf(dev, "Allocated the following %d IOCap key ids\n", key_suballoc->n_keys);

	// Search for the right number of keys in the bitmask, then save which ones we want to take.
	// Don't count them as taken yet, because we might not find them all.
	for (uint8_t i = 0; i < key_suballoc->n_keys; i++) {
		// TODO go back to rolling my own bitset, this one doesn't even use _clz
		uint16_t key_plus_one = BIT_FFS(256, &sc->available_keys);

		KASSERT(key_plus_one > 0, ("iocap_keymngr_alloc_key_ids had BIT_COUNT space but didn't find a set bit\n"));

		uint8_t key_id = key_plus_one - 1;
		key_suballoc->key_datas[i].key_id = key_id;
		BIT_CLR(256, key_id, &sc->available_keys);
		device_printf(dev, "key #%d\n", key_id);
	}

	// Unlock key manager
	mtx_unlock(&sc->iocap_keymngr_mtx);

	return 0;
}

// Fill the key data for this id with random data so it can be used to mint iocaps.
//
// Assumes the relevant key allocator mutex is held.
static void iocap_keymngr_init_key(device_t dev, struct iocap_key_state* key_state)
{
	struct iocap_keymngr_softc *sc;

	sc = device_get_softc(dev);

	// TODO do we need to take/release the lock on the key manager here?
	// From a strict perspective, probably, but all we're trying to do here is use the bus_space.
	// That bus_space is constant unless we're in the middle of tearing down the whole thing anyway.

	KASSERT(!key_state->active,
		("Key %d must not already be active", key_state->key_id));

	// Take random data
	arc4random_buf(key_state->key_data, 16);
	key_state->active = true;
	// Write the key data into the MMIO device
	// key_data is aligned to 8-bytes so we can cast the pointer to uint64
	// bus_space_write_multi etc. are not implemented for this specific bus_space... bleh
	for (int i = 0; i < 2; i++) {
		uint64_t* key_as_64bits = (uint64_t*)key_state->key_data;
		uint64_t key_i = key_as_64bits[i];
		bus_space_write_8(sc->bst, sc->bsh,
			0x1000 + (key_state->key_id << 4) + (i << 3),
			key_i);
	}

	// TODO memory barrier needed?
	mb();
	// Set the key status in the MMIO device as 1
	bus_space_write_8(sc->bst, sc->bsh, 0x0 + (key_state->key_id << 4), 1);
}

// Clear out the key data for this ID.
//
// Assumes the relevant key suballocator mutex is held.
static void iocap_keymngr_clear_key(device_t dev, struct iocap_key_state* key_state)
{
	struct iocap_keymngr_softc *sc;

	sc = device_get_softc(dev);

	KASSERT(key_state->active,
		("Key %d must be active to clear it", key_state->key_id));

	uint8_t key_id = key_state->key_id;

	// Tell device to start revoking as early as possible
	bus_space_write_8(sc->bst, sc->bsh, 0x0 + (key_id << 4), 0);
	// Clear data out
	memset(key_state->key_data, 0, 16);
	key_state->active = false;
	// Check the MMIO device has actually revoked
	while (bus_space_read_8(sc->bst, sc->bsh, 0x0 + (key_id << 4)) != 0) {
		// wait until the MMIO device confirms revocation with
		// key status == 0
	}
}

// Clear data for all given key IDs in a key suballocator and mark them as not-allocated in the device so other tags can reuse them.
// TODO need some way to ensure this doesn't leave a key open and usable,
//
// Assumes the relevant key suballocator mutex is held.
// Takes the key manager mutex.
static void iocap_keymngr_free_key_ids(device_t dev, struct iocap_key_suballocator* key_suballoc)
{
	struct iocap_keymngr_softc *sc;

	sc = device_get_softc(dev);

	// Lock key manager
	mtx_lock(&sc->iocap_keymngr_mtx);

	KASSERT(key_suballoc->n_keys + BIT_COUNT(256, &sc->available_keys) <= 256,
		("Inconsistency: somehow we are freeing %d key IDs but already have %ld available.",
			key_suballoc->n_keys, BIT_COUNT(256, &sc->available_keys)));

	for (int i = 0; i < key_suballoc->n_keys; i++) {
		uint8_t key_id;

		key_id = key_suballoc->key_datas[i].key_id;

		BIT_SET(256, key_id, &sc->available_keys);
	}

	// Unlock key manager
	mtx_unlock(&sc->iocap_keymngr_mtx);
}


// Assign a key within a tag for a map.
// Increment the refcount for that key on the given tag.
// If the refcount increases from zero for that key call iocap_keymngr_init_key.
//
// Takes the key allocator mutex for the tag.
static int
iocap_enabled_tag_assign_key(bus_dma_iocap_enabled_tag_t tag,
		bus_iocap_dmamap_t map)
{
	struct iocap_key_suballocator* key_suballoc = &tag->key_suballoc;

	// Take the key alloc lock
	mtx_lock(&key_suballoc->mtx);

	// TODO depending on revocation strategy we should change this
	uint8_t nth_key_of_tag = 0xFF;

	switch (key_suballoc->params.mode) {
	case iocap_revoke_when_no_mappings_unsafe: {
		nth_key_of_tag = 0;
		uint64_t refcount = key_suballoc->single_refcounted_key.active_mappings + 1;
		KASSERT(refcount != 0,
			("%s - overflowed refcount for single refcounted key of tag %p",
				device_get_name(tag->iocap_keymngr), tag));
		if (refcount == 1) {
			struct iocap_key_state* key_state = &key_suballoc->key_datas[nth_key_of_tag];
			// device_printf(tag->iocap_keymngr, "Activating IOCap key %d\n", key_state->key_id);
			iocap_keymngr_init_key(tag->iocap_keymngr, key_state);
		}
		key_suballoc->single_refcounted_key.active_mappings = refcount;
		key_suballoc->single_refcounted_key.total_mappings++;
		break;
	}
	case iocap_rolling_epoch_x4: {
		if (key_suballoc->rolling_epochs_x4.is_full) {
			device_printf(tag->iocap_keymngr, "Can't allocate key, device full!\n");

			// Unlock the key alloc lock
			mtx_unlock(&key_suballoc->mtx);

			return ENOSPC;
		}
		nth_key_of_tag = key_suballoc->rolling_epochs_x4.insertion_key;

		// key_suballoc->rolling_epochs_x4.key_stats[insertion_key].bytes_mapped_left -= map->base_map->;
		uint64_t refcount = key_suballoc->rolling_epochs_x4.key_stats[nth_key_of_tag].active_mappings + 1;
		KASSERT(refcount != 0,
			("%s - overflowed refcount for single refcounted key of tag %p",
				device_get_name(tag->iocap_keymngr), tag));
		// If we're setting the refcount to 1, the key must not have been active before - reactivate it
		if (refcount == 1) {
			struct iocap_key_state* key_state = &key_suballoc->key_datas[nth_key_of_tag];
			KASSERT(key_state->active == false, ("Insertion key [%d] had no references but an active key", nth_key_of_tag));
			// device_printf(tag->iocap_keymngr, "Activating IOCap key %d\n", key_state->key_id);
			iocap_keymngr_init_key(tag->iocap_keymngr, key_state);
		}
		key_suballoc->rolling_epochs_x4.key_stats[nth_key_of_tag].active_mappings = refcount;

		// Have we filled this key up? Either move on, or the whole thing is full
		key_suballoc->rolling_epochs_x4.key_stats[nth_key_of_tag].num_mappings_left--;
		if (key_suballoc->rolling_epochs_x4.key_stats[nth_key_of_tag].num_mappings_left == 0) {
			uint8_t next_key = 0xFF;
			bool found_next_key = false;
			for (uint8_t i = 1; i < key_suballoc->n_keys; i++) {
				next_key = (nth_key_of_tag + i) % key_suballoc->n_keys;
				if (key_suballoc->rolling_epochs_x4.key_stats[next_key].num_mappings_left > 0) {
					found_next_key = true;
					break;
				}
			}
			if (found_next_key) {
				// device_printf(tag->iocap_keymngr, "Rolling key forward from %d to %d\n", nth_key_of_tag, next_key);
				key_suballoc->rolling_epochs_x4.insertion_key = next_key;
			} else {
				device_printf(tag->iocap_keymngr, "Can't roll key forward from %d, device full!\n", nth_key_of_tag);
				key_suballoc->rolling_epochs_x4.is_full = true;
			}
		}
		break;
	}
	}

	map->nth_key_of_tag = nth_key_of_tag; // TODO KASSERT nth_key_of_tag != 0xFF
	KASSERT(map->state == iocap_dmamap_loaded,
			("Tried to complete a DMA map after it was completed %d", map->state));
	map->state = iocap_dmamap_key_selected;

	// Unlock the key alloc lock
	mtx_unlock(&key_suballoc->mtx);

	return 0;
}

static void flush_quarantines(struct iocap_keymngr_quarantine_queue* list)
{
	struct iocap_keymngr_quarantined_mem_cb* entry;
	while (!SLIST_EMPTY(list)) {
		entry = SLIST_FIRST(list);
		SLIST_REMOVE_HEAD(list, entries);
		entry->cb(entry->cb_arg1, entry->cb_arg2);
		free(entry, M_IOCAP_DMAMAP_QUARANTINE);
	}
}

// Decrement the refcount for a map's key on a given tag and transitions the map to iocap_dmamap_unloaded.
// If the map is in iocap_dmamap_loaded, does nothing.
// If the map is otherwise not in iocap_dmamap_key_selected, asserts.
//
// When the refcount hits zero, call iocap_keymngr_clear_key() to clear the key data, and call all quarantine callbacks, but keep the key index allocated.
//
// If cb is not NULL and the refcount hits zero, calls cb(cb_arg1, cb_arg2) after the other quarantine callbacks.
// If cb is not NULL and the refcount doesn't hit 0, enqueues (cb, cb_arg1, cb_arg2) to the relevant quarantine queue.
//
// Takes the key allocator mutex for the tag if the map is in iocap_dmamap_key_selected.
static int
iocap_enabled_tag_unassign_key(bus_dma_iocap_enabled_tag_t tag,
		bus_iocap_dmamap_t map, iocap_keymngr_bus_dmamap_unload2_cb cb,
		void* cb_arg1, void* cb_arg2)
{
	struct iocap_key_suballocator* key_suballoc = &tag->key_suballoc;

	if (map->state == iocap_dmamap_loaded) {
		// No key assigned
		map->state = iocap_dmamap_unloaded;
		return 0;
	}

	KASSERT(map->state == iocap_dmamap_key_selected,
			("Tried to unload a DMA map when it was not loaded"));

	// Take the key alloc lock
	mtx_lock(&key_suballoc->mtx);

	switch (key_suballoc->params.mode) {
	case iocap_revoke_when_no_mappings_unsafe: {
		uint64_t refcount = key_suballoc->single_refcounted_key.active_mappings - 1;
		if (refcount == 0) {
			if (key_suballoc->single_refcounted_key.total_mappings > 1) {
				device_printf(tag->iocap_keymngr,
					"Deactivating IOCap key %d with max refcount %zu\n",
					key_suballoc->key_datas[0].key_id, key_suballoc->single_refcounted_key.total_mappings);
			}

			struct iocap_key_state* key_state = &key_suballoc->key_datas[0];
			// device_printf(tag->iocap_keymngr, "Deactivating IOCap key %d\n", key_state->key_id);
			iocap_keymngr_clear_key(tag->iocap_keymngr, key_state);

			flush_quarantines(&key_suballoc->single_refcounted_key.quarantine);
			if (cb != NULL) {
				cb(cb_arg1, cb_arg2);
			}

			// Reset the key state
			memset(&key_suballoc->single_refcounted_key, 0, sizeof(key_suballoc->single_refcounted_key));
		} else if (cb != NULL) {
			struct iocap_keymngr_quarantined_mem_cb* cb_entry = malloc(
				sizeof(struct iocap_keymngr_quarantined_mem_cb),
				M_IOCAP_DMAMAP_QUARANTINE, M_NOWAIT);
			cb_entry->cb = cb;
			cb_entry->cb_arg1 = cb_arg1;
			cb_entry->cb_arg2 = cb_arg2;
			SLIST_INSERT_HEAD(&key_suballoc->single_refcounted_key.quarantine, cb_entry, entries);
		}
		key_suballoc->single_refcounted_key.active_mappings = refcount;
		break;
	}
	case iocap_rolling_epoch_x4: {
		uint8_t key_id = map->nth_key_of_tag;

		// Decrement the active refcount
		uint64_t refcount = key_suballoc->rolling_epochs_x4.key_stats[key_id].active_mappings - 1;

		// If the key is now unused, we need to
		// - Clear the key
		// - Flush any quarantines
		// - Call this quarantine callback
		// - If the tag is_full, set the insertion_key and set is_full = false
		// - Reset the key state
		if (refcount == 0) {
			// if (key_suballoc->rolling_epochs_x4.key_stats[key_id].num_mappings_left =) {
			// 	device_printf(tag->iocap_keymngr,
			// 		"Deactivating IOCap key %d with max refcount %zu\n",
			// 		key_suballoc->key_datas[0].key_id, key_suballoc->rolling_epochs_x4.key_stats[key_id].total_mappings);
			// }

			// Clear the key
			struct iocap_key_state* key_state = &key_suballoc->key_datas[key_id];
			// device_printf(tag->iocap_keymngr, "Deactivating IOCap key %d\n", key_state->key_id);
			iocap_keymngr_clear_key(tag->iocap_keymngr, key_state);

			// Flush any quarantines
			flush_quarantines(&key_suballoc->rolling_epochs_x4.key_stats[key_id].quarantine);
			// Call this quarantine callback
			if (cb != NULL) {
				cb(cb_arg1, cb_arg2);
			}

			// If the tag is_full, set the insertion_key and set is_full = false
			if (key_suballoc->rolling_epochs_x4.is_full) {
				key_suballoc->rolling_epochs_x4.insertion_key = key_id;
				key_suballoc->rolling_epochs_x4.is_full = false;
			}

			// Reset the key state
			key_suballoc->rolling_epochs_x4.key_stats[key_id].active_mappings = 0;
			key_suballoc->rolling_epochs_x4.key_stats[key_id].num_mappings_left = key_suballoc->params.params.rolling_epoch.max_num_mappings_per_epoch;
		} else if (cb != NULL) {
			// Otherwise just add to the quarantine
			struct iocap_keymngr_quarantined_mem_cb* cb_entry = malloc(
				sizeof(struct iocap_keymngr_quarantined_mem_cb),
				M_IOCAP_DMAMAP_QUARANTINE, M_NOWAIT);
			cb_entry->cb = cb;
			cb_entry->cb_arg1 = cb_arg1;
			cb_entry->cb_arg2 = cb_arg2;
			SLIST_INSERT_HEAD(&key_suballoc->rolling_epochs_x4.key_stats[key_id].quarantine, cb_entry, entries);
		}
		key_suballoc->rolling_epochs_x4.key_stats[key_id].active_mappings = refcount;
		break;
	}
	}

	map->nth_key_of_tag = 0xFF;
	map->state = iocap_dmamap_unloaded;

	// Unlock the key alloc lock
	mtx_unlock(&key_suballoc->mtx);

	return 0;
}

static int
refinable_tag_create(bus_dma_tag_t parent,
		bus_size_t alignment, bus_addr_t boundary, bus_addr_t lowaddr,
		bus_addr_t highaddr, bus_size_t maxsize, int nsegments,
		bus_size_t maxsegsz, int flags, bus_dma_lock_t *lockfunc,
		void *lockfuncarg, bus_dma_tag_t *dmat)
{
	struct bus_dma_iocap_refinable_tag *refine_parent;
	struct bus_dma_impl *parent_base_impl;
	int error;

	refine_parent = (struct bus_dma_iocap_refinable_tag *)parent;
	parent_base_impl = ((struct bus_dma_tag_common *)refine_parent->
		base_tag)->impl;

	// Create a new refinable tag with a (base = refined(parent.base))
	struct bus_dma_iocap_refinable_tag *new_tag = malloc(
			sizeof(struct bus_dma_iocap_refinable_tag),
			M_IOCAP_DMATAG, M_ZERO | M_NOWAIT);
	new_tag->iocap_keymngr = refine_parent->iocap_keymngr;
	new_tag->common.impl = &bus_dma_iocap_refinable_tag_impl;

	error = parent_base_impl->tag_create(
		refine_parent->base_tag, alignment, boundary, lowaddr,
		highaddr, maxsize, nsegments, maxsegsz, flags, lockfunc,
		lockfuncarg, &new_tag->base_tag
	);

	if (error != 0) {
		free(new_tag, M_IOCAP_DMATAG);
		device_printf(refine_parent->iocap_keymngr, "failed to tag_create from base to make new refinable tag\n");
		return error;
	}

	*dmat = (bus_dma_tag_t)new_tag;

	return 0;
}

static int
refinable_tag_destroy(bus_dma_tag_t dmat)
{
	struct bus_dma_iocap_refinable_tag *refine_dmat;
	struct bus_dma_impl *base_impl;
	int error;

	refine_dmat = (struct bus_dma_iocap_refinable_tag *)dmat;
	base_impl = ((struct bus_dma_tag_common *)refine_dmat->base_tag)->impl;

	error = base_impl->tag_destroy(refine_dmat->base_tag);

	free(refine_dmat, M_IOCAP_DMATAG);

	return error;
}

static int
refinable_map_create(bus_dma_tag_t dmat, int flags, bus_dmamap_t *mapp)
{
	struct bus_dma_iocap_refinable_tag *refine_dmat;
	struct bus_dma_impl *base_impl;

	refine_dmat = (struct bus_dma_iocap_refinable_tag *)dmat;
	base_impl = ((struct bus_dma_tag_common *)refine_dmat->base_tag)->impl;

	return base_impl->map_create(refine_dmat->base_tag, flags, mapp);
}

static int
refinable_map_destroy(bus_dma_tag_t dmat, bus_dmamap_t map)
{
	struct bus_dma_iocap_refinable_tag *refine_dmat;
	struct bus_dma_impl *base_impl;

	refine_dmat = (struct bus_dma_iocap_refinable_tag *)dmat;
	base_impl = ((struct bus_dma_tag_common *)refine_dmat->base_tag)->impl;

	return base_impl->map_destroy(refine_dmat->base_tag, map);
}

static int
refinable_mem_alloc(bus_dma_tag_t dmat, void **vaddr, int flags,
		bus_dmamap_t *mapp)
{
	struct bus_dma_iocap_refinable_tag *refine_dmat;
	struct bus_dma_impl *base_impl;

	refine_dmat = (struct bus_dma_iocap_refinable_tag *)dmat;
	base_impl = ((struct bus_dma_tag_common *)refine_dmat->base_tag)->impl;

	return base_impl->mem_alloc(refine_dmat->base_tag, vaddr, flags, mapp);
}

static void
refinable_mem_free(bus_dma_tag_t dmat, void *vaddr, bus_dmamap_t map)
{
	struct bus_dma_iocap_refinable_tag *refine_dmat;
	struct bus_dma_impl *base_impl;

	refine_dmat = (struct bus_dma_iocap_refinable_tag *)dmat;
	base_impl = ((struct bus_dma_tag_common *)refine_dmat->base_tag)->impl;

	return base_impl->mem_free(refine_dmat->base_tag, vaddr, map);
}

static int
refinable_load_ma(bus_dma_tag_t dmat, bus_dmamap_t map,
		struct vm_page **ma, bus_size_t tlen, int ma_offs, int flags,
		bus_dma_segment_t *segs, int *segp)
{
	struct bus_dma_iocap_refinable_tag *refine_dmat;
	struct bus_dma_impl *base_impl;

	refine_dmat = (struct bus_dma_iocap_refinable_tag *)dmat;
	base_impl = ((struct bus_dma_tag_common *)refine_dmat->base_tag)->impl;

	return base_impl->load_ma(refine_dmat->base_tag, map, ma, tlen, ma_offs,
			flags, segs, segp);
}

static int
refinable_load_phys(bus_dma_tag_t dmat, bus_dmamap_t map,
		vm_paddr_t buf, bus_size_t buflen, int flags,
		bus_dma_segment_t *segs, int *segp)
{
	struct bus_dma_iocap_refinable_tag *refine_dmat;
	struct bus_dma_impl *base_impl;

	refine_dmat = (struct bus_dma_iocap_refinable_tag *)dmat;
	base_impl = ((struct bus_dma_tag_common *)refine_dmat->base_tag)->impl;

	return base_impl->load_phys(refine_dmat->base_tag, map, buf, buflen,
			flags, segs, segp);
}

static int
refinable_load_buffer(bus_dma_tag_t dmat, bus_dmamap_t map,
		void *buf, bus_size_t buflen, struct pmap *pmap, int flags,
		bus_dma_segment_t *segs, int *segp)
{
	struct bus_dma_iocap_refinable_tag *refine_dmat;
	struct bus_dma_impl *base_impl;

	refine_dmat = (struct bus_dma_iocap_refinable_tag *)dmat;
	base_impl = ((struct bus_dma_tag_common *)refine_dmat->base_tag)->impl;

	return base_impl->load_buffer(refine_dmat->base_tag, map, buf, buflen,
			pmap, flags, segs, segp);
}

static void
refinable_map_waitok(bus_dma_tag_t dmat, bus_dmamap_t map,
		struct memdesc *mem, bus_dmamap_callback_t *callback,
		void *callback_arg)
{
	struct bus_dma_iocap_refinable_tag *refine_dmat;
	struct bus_dma_impl *base_impl;

	refine_dmat = (struct bus_dma_iocap_refinable_tag *)dmat;
	base_impl = ((struct bus_dma_tag_common *)refine_dmat->base_tag)->impl;

	base_impl->map_waitok(refine_dmat->base_tag, map, mem, callback,
			callback_arg);
}

static bus_dma_segment_t *
refinable_map_complete(bus_dma_tag_t dmat, bus_dmamap_t map,
		bus_dma_segment_t *segs, int nsegs, int error)
{
	struct bus_dma_iocap_refinable_tag *refine_dmat;
	struct bus_dma_impl *base_impl;

	refine_dmat = (struct bus_dma_iocap_refinable_tag *)dmat;
	base_impl = ((struct bus_dma_tag_common *)refine_dmat->base_tag)->impl;

	return base_impl->map_complete(refine_dmat->base_tag, map, segs, nsegs,
			error);
}

static void
refinable_map_unload(bus_dma_tag_t dmat, bus_dmamap_t map)
{
	struct bus_dma_iocap_refinable_tag *refine_dmat;
	struct bus_dma_impl *base_impl;

	refine_dmat = (struct bus_dma_iocap_refinable_tag *)dmat;
	base_impl = ((struct bus_dma_tag_common *)refine_dmat->base_tag)->impl;

	base_impl->map_unload(refine_dmat->base_tag, map);
}

static void
refinable_map_sync(bus_dma_tag_t dmat, bus_dmamap_t map,
		bus_dmasync_op_t op)
{
	struct bus_dma_iocap_refinable_tag *refine_dmat;
	struct bus_dma_impl *base_impl;

	refine_dmat = (struct bus_dma_iocap_refinable_tag *)dmat;
	base_impl = ((struct bus_dma_tag_common *)refine_dmat->base_tag)->impl;

	base_impl->map_sync(refine_dmat->base_tag, map, op);
}

struct bus_dma_impl bus_dma_iocap_refinable_tag_impl = {
	.tag_create = refinable_tag_create,
	.tag_destroy = refinable_tag_destroy,
	.map_create = refinable_map_create,
	.map_destroy = refinable_map_destroy,
	.mem_alloc = refinable_mem_alloc,
	.mem_free = refinable_mem_free,
	.load_phys = refinable_load_phys,
	.load_buffer = refinable_load_buffer,
	.load_ma = refinable_load_ma,
	.map_waitok = refinable_map_waitok,
	.map_complete = refinable_map_complete,
	.map_unload = refinable_map_unload,
	.map_sync = refinable_map_sync
};

// malloc-s a struct bus_iocap_dmamap, populating all fields except the base_map.
static bus_iocap_dmamap_t
_iocap_enabled_create_map_common(bus_dma_iocap_enabled_tag_t tag)
{
	bus_iocap_dmamap_t map;

	map = malloc(sizeof(*map), M_IOCAP_DMAMAP, M_NOWAIT | M_ZERO);
	map->magic = BUS_IOCAP_DMAMAP_MAGIC;
	map->state = iocap_dmamap_unloaded;
	map->tag = tag;
	map->nth_key_of_tag = 0xFF; // invalid, not set yet because we are unloaded

	// TODO does each map need a lock?

	return map;
}

// free-s a struct bus_iocap_dmamap, assuming the base_map has already been freed.
// If it still has contents loaded, it decrements the refcount for the relevant key.
// This should never happen.
// TODO need to make sure the transisiton from loaded -> unloaded and decrementing the key is altogether atomic
static void
_iocap_enabled_destroy_map_common(bus_dma_iocap_enabled_tag_t tag,
		bus_iocap_dmamap_t map)
{
	if (map->state == iocap_dmamap_loaded) {
		device_printf(tag->iocap_keymngr, "Destroying an IOCap map that hadn't been unloaded?");
		iocap_enabled_tag_unassign_key(tag, map, NULL, NULL, NULL);
	}

	// TODO if we end up putting a lock in each dmamap like IOMMU does, destroy it here

	free(map, M_IOCAP_DMAMAP);
}

static int
iocap_enabled_tag_create(
		bus_dma_tag_t parent,
		bus_size_t alignment, bus_addr_t boundary, bus_addr_t lowaddr,
		bus_addr_t highaddr, bus_size_t maxsize, int nsegments,
		bus_size_t maxsegsz, int flags, bus_dma_lock_t *lockfunc,
		void *lockfuncarg, bus_dma_tag_t *dmat)
{
	// We do not allow refining iocap_enabled tags,
	// because then there would be multiple iocap_enabled tags using the same key IDs.
	// This would make managing those key IDs more complicated.
	// Right now it's simple, in iocap_enabled_tag_destroy we can free all the key IDs associated with the tag.
	return EOPNOTSUPP;
}

static int
iocap_enabled_tag_destroy(bus_dma_tag_t dmat)
{
	struct bus_dma_iocap_enabled_tag *iocap_dmat;
	struct bus_dma_impl *base_impl;
	int error;

	iocap_dmat = (struct bus_dma_iocap_enabled_tag *)dmat;

	// device_printf(iocap_dmat->iocap_keymngr, "%s\n", __func__);

	base_impl = ((struct bus_dma_tag_common *)iocap_dmat->base_tag)->impl;
	error = 0;

	// Lock the key allocator mutex
	mtx_lock(&iocap_dmat->key_suballoc.mtx);

	// This implicitly makes all mappings from this tag inaccessible :)
	iocap_keymngr_free_key_ids(iocap_dmat->iocap_keymngr,
			&iocap_dmat->key_suballoc);

	// Unlock and destroy the key allocator mutex now we definitely aren't using it
	mtx_unlock(&iocap_dmat->key_suballoc.mtx);
	mtx_destroy(&iocap_dmat->key_suballoc.mtx);

	error = base_impl->tag_destroy(iocap_dmat->base_tag);

	free(iocap_dmat, M_IOCAP_DMAMAP);

	return error;
}

static int
iocap_enabled_map_create(bus_dma_tag_t dmat, int flags, bus_dmamap_t *mapp)
{
	struct bus_dma_iocap_enabled_tag *iocap_dmat;
	struct bus_dma_impl *base_impl;
	struct bus_iocap_dmamap *iocap_map;
	int error;

	iocap_dmat = (struct bus_dma_iocap_enabled_tag *)dmat;

	// device_printf(iocap_dmat->iocap_keymngr, "%s\n", __func__);

	base_impl = ((struct bus_dma_tag_common *)iocap_dmat->base_tag)->impl;
	iocap_map = _iocap_enabled_create_map_common(iocap_dmat);
	error = 0;

	error = base_impl->map_create(iocap_dmat->base_tag, flags,
			&iocap_map->base_map);
	if (error) {
		_iocap_enabled_destroy_map_common(iocap_dmat, iocap_map);
		return error;
	}

	*mapp = (bus_dmamap_t)iocap_map;

	return 0;
}

static int
iocap_enabled_map_destroy(bus_dma_tag_t dmat, bus_dmamap_t map)
{
	struct bus_dma_iocap_enabled_tag *iocap_dmat;
	struct bus_dma_impl *base_impl;
	struct bus_iocap_dmamap *iocap_map;
	int error;

	iocap_dmat = (struct bus_dma_iocap_enabled_tag *)dmat;

	// device_printf(iocap_dmat->iocap_keymngr, "%s\n", __func__);

	base_impl = ((struct bus_dma_tag_common *)iocap_dmat->base_tag)->impl;
	iocap_map = (struct bus_iocap_dmamap *)map;

	error = base_impl->map_destroy(iocap_dmat->base_tag,
			iocap_map->base_map);

	_iocap_enabled_destroy_map_common(iocap_dmat, iocap_map);

	return error;
}

static int
iocap_enabled_mem_alloc(bus_dma_tag_t dmat, void **vaddr, int flags,
		bus_dmamap_t *mapp)
{
	struct bus_dma_iocap_enabled_tag *iocap_dmat;
	struct bus_dma_impl *base_impl;
	struct bus_iocap_dmamap *iocap_map;

	iocap_dmat = (struct bus_dma_iocap_enabled_tag *)dmat;

	// device_printf(iocap_dmat->iocap_keymngr, "%s\n", __func__);

	base_impl = ((struct bus_dma_tag_common *)iocap_dmat->base_tag)->impl;
	iocap_map = _iocap_enabled_create_map_common(iocap_dmat);

	int error = 0;

	// use base_impl to allocate the memory and fill in the base_map
	error = base_impl->mem_alloc(iocap_dmat->base_tag, vaddr, flags,
			&iocap_map->base_map);
	if (error) {
		return error;
	}

	*mapp = (bus_dmamap_t)iocap_map;

	return 0;
}

static void
iocap_enabled_mem_free(bus_dma_tag_t dmat, void *vaddr, bus_dmamap_t map)
{
	struct bus_dma_iocap_enabled_tag *iocap_dmat;
	struct bus_dma_impl *base_impl;
	struct bus_iocap_dmamap *iocap_map;

	iocap_dmat = (struct bus_dma_iocap_enabled_tag *)dmat;

	// device_printf(iocap_dmat->iocap_keymngr, "%s\n", __func__);

	base_impl = ((struct bus_dma_tag_common *)iocap_dmat->base_tag)->impl;
	iocap_map = (struct bus_iocap_dmamap *)map;

	// Use base_impl to free the memory it allocated in mem_alloc
	base_impl->mem_free(iocap_dmat->base_tag, vaddr, iocap_map->base_map);

	// do the rest of the IOCap-specific destruction
	_iocap_enabled_destroy_map_common(iocap_dmat, iocap_map);
}

static int
iocap_enabled_load_ma(bus_dma_tag_t dmat, bus_dmamap_t map,
		struct vm_page **ma, bus_size_t tlen, int ma_offs, int flags,
		bus_dma_segment_t *segs, int *segp)
{
	struct bus_dma_iocap_enabled_tag *iocap_dmat;
	struct bus_dma_impl *base_impl;
	struct bus_iocap_dmamap *iocap_map;

	iocap_dmat = (struct bus_dma_iocap_enabled_tag *)dmat;

	// device_printf(iocap_dmat->iocap_keymngr, "%s\n", __func__);

	base_impl = ((struct bus_dma_tag_common *)iocap_dmat->base_tag)->impl;
	iocap_map = (struct bus_iocap_dmamap *)map;

	KASSERT(iocap_map->state != iocap_dmamap_loaded,
	("Tried to load more entries into a DMA map after it was completed"
	));
	iocap_map->state = iocap_dmamap_loading;

	return base_impl->load_ma(iocap_dmat->base_tag, iocap_map->base_map, ma,
			tlen, ma_offs, flags, segs, segp);
}

static int
iocap_enabled_load_phys(bus_dma_tag_t dmat, bus_dmamap_t map,
		vm_paddr_t buf, bus_size_t buflen, int flags,
		bus_dma_segment_t *segs, int *segp)
{
	struct bus_dma_iocap_enabled_tag *iocap_dmat;
	struct bus_dma_impl *base_impl;
	struct bus_iocap_dmamap *iocap_map;

	iocap_dmat = (struct bus_dma_iocap_enabled_tag *)dmat;

	// device_printf(iocap_dmat->iocap_keymngr, "%s\n", __func__);

	base_impl = ((struct bus_dma_tag_common *)iocap_dmat->base_tag)->impl;
	iocap_map = (struct bus_iocap_dmamap *)map;

	KASSERT(iocap_map->state != iocap_dmamap_loaded,
	("Tried to load more entries into a DMA map after it was completed"
	));
	iocap_map->state = iocap_dmamap_loading;

	return base_impl->load_phys(iocap_dmat->base_tag, iocap_map->base_map,
			buf, buflen, flags, segs, segp);
}

static int
iocap_enabled_load_buffer(bus_dma_tag_t dmat, bus_dmamap_t map,
		void *buf, bus_size_t buflen, struct pmap *pmap, int flags,
		bus_dma_segment_t *segs, int *segp)
{
	struct bus_dma_iocap_enabled_tag *iocap_dmat;
	struct bus_dma_impl *base_impl;
	struct bus_iocap_dmamap *iocap_map;

	iocap_dmat = (struct bus_dma_iocap_enabled_tag *)dmat;

	// device_printf(iocap_dmat->iocap_keymngr, "%s\n", __func__);

	base_impl = ((struct bus_dma_tag_common *)iocap_dmat->base_tag)->impl;
	iocap_map = (struct bus_iocap_dmamap *)map;

	KASSERT(iocap_map->state != iocap_dmamap_loaded,
	("Tried to load more entries into a DMA map after it was completed"
	));
	iocap_map->state = iocap_dmamap_loading;

	// device_printf(iocap_dmat->iocap_keymngr, "iocap_enabled_load_buffer buf %p buflen 0x%lx\n", buf, buflen);

	// TODO the default riscv bounce buffer implementation may pad the dmasegments out to 4KiB multiples.
	// This is only in the case where addr_needs_bounce(dmat, curaddr): see busdma_bounce.c:673
	// This discards the (very important!) raw size information that we need
	// to generate tight IOcaps.
	// This may be fixable by changing the alignment restrictions implied by the virtio blk driver,
	// which control the outcome of addr_needs_bounce.
	return base_impl->load_buffer(iocap_dmat->base_tag, iocap_map->base_map,
			buf, buflen, pmap, flags, segs, segp);
}

static void
iocap_enabled_map_waitok(bus_dma_tag_t dmat, bus_dmamap_t map,
		struct memdesc *mem, bus_dmamap_callback_t *callback,
		void *callback_arg)
{
	struct bus_dma_iocap_enabled_tag *iocap_dmat;
	struct bus_dma_impl *base_impl;
	struct bus_iocap_dmamap *iocap_map;

	iocap_dmat = (struct bus_dma_iocap_enabled_tag *)dmat;

	// device_printf(iocap_dmat->iocap_keymngr, "%s\n", __func__);

	base_impl = ((struct bus_dma_tag_common *)iocap_dmat->base_tag)->impl;
	iocap_map = (struct bus_iocap_dmamap *)map;

	KASSERT(iocap_map->state != iocap_dmamap_loaded,
			("Tried to waitok on a DMA map after it was completed"))
	;
	iocap_map->state = iocap_dmamap_loading;

	base_impl->map_waitok(iocap_dmat->base_tag, iocap_map->base_map, mem,
			callback, callback_arg);
}

/// iocap_enabled_map_sync and iocap_enabled_map_unload are two sides of the same coin.
/// They are the keymngr-side functions that actually increment/decrement the refcounts of
/// keys in the dmatag.
/// Note that these are *not* the functions that mint IOCaps! Because there may be multiple segments per
/// dmamap, and there may be metadata from another layer that needs to get stuffed into the IOCap (e.g. virtio next ptr)
/// so that happens later. This is *just* selecting a key and incrementing a refcount, or decrementing that refcount.
///
/// Reason this isn't iocap_enabled_map_complete:
/// The virtio_blk driver (and maybe others) expect to be able to atomically load-then-detect-error-then-unload
/// without causing problems. Perhaps bus_dmamap_sync(, BUS_DMASYNC_PRE{READ,WRITE}) would be a good alternate choice,
/// as it's required --- "Before and after each DMA transaction, bus_dmamap_sync() must be used
/// to ensure that the correct data	is used	by the DMA engine and the CPU." --- and is less likely to be done if
/// an error has been detected. There is no guarantee that this function doesn't block, either...

// struct iocap_keymngr_select_key_and_incref_action {
// 	struct bus_iocap_dmamap* iocap_map;
// };
//
// struct iocap_keymngr_decref_key_action {
// 	uint8_t key_index;
// };
// TODO look back at this

// Finish loading memory into the IOCap. Allocates a key from the parent tag
// now that we are exposing memory.
static bus_dma_segment_t *
iocap_enabled_map_complete(bus_dma_tag_t dmat, bus_dmamap_t map,
		bus_dma_segment_t *segs, int nsegs, int error)
{
	struct bus_dma_iocap_enabled_tag *iocap_dmat;
	struct bus_dma_impl *base_impl;
	struct bus_iocap_dmamap *iocap_map;

	iocap_dmat = (struct bus_dma_iocap_enabled_tag *)dmat;

	// device_printf(iocap_dmat->iocap_keymngr, "%s\n", __func__);

	base_impl = ((struct bus_dma_tag_common *)iocap_dmat->base_tag)->impl;
	iocap_map = (struct bus_iocap_dmamap *)map;

	// Transition the map to "loaded" state
	iocap_map->state = iocap_dmamap_loaded;

	return base_impl->map_complete(iocap_dmat->base_tag,
			iocap_map->base_map, segs, nsegs, error);
}

static void
iocap_enabled_map_unload(bus_dma_tag_t dmat, bus_dmamap_t map)
{
	struct bus_dma_iocap_enabled_tag *iocap_dmat;
	struct bus_dma_impl *base_impl;
	struct bus_iocap_dmamap *iocap_map;

	iocap_dmat = (struct bus_dma_iocap_enabled_tag *)dmat;

	// device_printf(iocap_dmat->iocap_keymngr, "%s\n", __func__);

	base_impl = ((struct bus_dma_tag_common *)iocap_dmat->base_tag)->impl;
	iocap_map = (struct bus_iocap_dmamap *)map;

	// Transitions map to unloaded
	iocap_enabled_tag_unassign_key(iocap_dmat, iocap_map, NULL, NULL, NULL);

	base_impl->map_unload(iocap_dmat->base_tag, iocap_map->base_map);
}

// version of bus_dmamap_unload with a callback, see iocap_keymngr.h
void
iocap_keymngr_bus_dmamap_unload2(bus_dma_iocap_enabled_tag_t iocap_dmat, bus_iocap_dmamap_t iocap_map,
	iocap_keymngr_bus_dmamap_unload2_cb on_unmapped, void* arg1, void* arg2)
{
	struct bus_dma_impl *base_impl;

	base_impl = ((struct bus_dma_tag_common *)iocap_dmat->base_tag)->impl;

	// Transitions map to unloaded
	iocap_enabled_tag_unassign_key(iocap_dmat, iocap_map, on_unmapped, arg1, arg2);

	base_impl->map_unload(iocap_dmat->base_tag, iocap_map->base_map);
}

static void
iocap_enabled_map_sync(bus_dma_tag_t dmat, bus_dmamap_t map,
		bus_dmasync_op_t op)
{
	struct bus_dma_iocap_enabled_tag *iocap_dmat;
	struct bus_dma_impl *base_impl;
	struct bus_iocap_dmamap *iocap_map;

	iocap_dmat = (struct bus_dma_iocap_enabled_tag *)dmat;

	// device_printf(iocap_dmat->iocap_keymngr, "%s\n", __func__);

	base_impl = ((struct bus_dma_tag_common *)iocap_dmat->base_tag)->impl;
	iocap_map = (struct bus_iocap_dmamap *)map;

	// // Transitions the map to "key-assigned" state if not already there
	// if (iocap_map->state == iocap_dmamap_loaded && (op == BUS_DMASYNC_PREREAD || op == BUS_DMASYNC_PREWRITE)) {
	// 	iocap_enabled_tag_assign_key(iocap_dmat, iocap_map);
	// }

	base_impl->map_sync(iocap_dmat->base_tag, iocap_map->base_map, op);
}

struct bus_dma_impl bus_dma_iocap_enabled_tag_impl = {
	.tag_create = iocap_enabled_tag_create,
	.tag_destroy = iocap_enabled_tag_destroy,
	.map_create = iocap_enabled_map_create,
	.map_destroy = iocap_enabled_map_destroy,
	.mem_alloc = iocap_enabled_mem_alloc,
	.mem_free = iocap_enabled_mem_free,
	.load_phys = iocap_enabled_load_phys,
	.load_buffer = iocap_enabled_load_buffer,
	.load_ma = iocap_enabled_load_ma,
	.map_waitok = iocap_enabled_map_waitok,
	.map_complete = iocap_enabled_map_complete,
	.map_unload = iocap_enabled_map_unload,
	.map_sync = iocap_enabled_map_sync
};

static bus_dma_tag_t
iocap_keymngr_get_dma_tag(device_t dev, device_t child)
{
	struct iocap_keymngr_softc *sc;

	sc = device_get_softc(dev);

	return (bus_dma_tag_t)&sc->base_refinable_tag;
}

bus_dma_iocap_refinable_tag_t
bus_dma_tag_iocap_refinable(bus_dma_tag_t tag)
{
	if (tag == NULL) {
		return NULL;
	}
	if (((struct bus_dma_tag_common *)tag)->impl == &
		bus_dma_iocap_refinable_tag_impl) {
		return (bus_dma_iocap_refinable_tag_t)tag;
	}
	return NULL;
}

int
bus_dma_tag_refine_to_iocap_group(bus_dma_iocap_refinable_tag_t tag,
		struct iocap_keymngr_revocation_params params,
		bus_dma_iocap_enabled_tag_t *out)
{
	if (!bus_dma_tag_iocap_refinable((bus_dma_tag_t)tag)) {
		return EPERM;
	}

	uint8_t n_keys;

	switch (params.mode) {
	case iocap_revoke_when_no_mappings_unsafe: {
		n_keys = 1;
		break;
	}
	case iocap_rolling_epoch_x4: {
		if (/* params.params.rolling_epoch.max_bytes_mapped_per_epoch == 0 && */ params.params.rolling_epoch.max_num_mappings_per_epoch == 0) {
			return EINVAL;
		}
		n_keys = 4;
		break;
	}
	default:
		return EINVAL;
	}

	struct bus_dma_iocap_enabled_tag *new_tag = malloc(
			sizeof(struct bus_dma_iocap_enabled_tag),
			M_IOCAP_DMAMAP, M_ZERO | M_NOWAIT);

	new_tag->common.impl = &bus_dma_iocap_enabled_tag_impl;
	new_tag->base_tag = tag->base_tag;
	new_tag->iocap_keymngr = tag->iocap_keymngr;
	new_tag->key_suballoc = (struct iocap_key_suballocator) {0};
	new_tag->key_suballoc.n_keys = n_keys;
	new_tag->key_suballoc.params = params;

	switch (params.mode) {
	case iocap_rolling_epoch_x4: {
		for (uint8_t i = 0; i < 4; i++)
			new_tag->key_suballoc.rolling_epochs_x4.key_stats[i].num_mappings_left =
				params.params.rolling_epoch.max_num_mappings_per_epoch;
		break;
	}
	case iocap_revoke_when_no_mappings_unsafe:
		// No extra initing
		break;
	}

	mtx_init(&new_tag->key_suballoc.mtx, "IOCap DMA Tag Key Allocation Mutex", NULL, MTX_DEF);

	int error = iocap_keymngr_alloc_key_ids(tag->iocap_keymngr,
			&new_tag->key_suballoc);
	if (error) {
		free(new_tag, M_IOCAP_DMAMAP);
		*out = NULL;
		return error;
	}

	*out = new_tag;

	return 0;
}

// Return 1 if the given bus_dmamap_t is an IOCap-able map - i.e. whether you can call bus_dmamap_mint_iocap
// or bus_dmamap_mint_virtio_iocap on it.
// Otherwise returns 0.
bus_iocap_dmamap_t
bus_dmamap_can_mint_iocap(bus_dmamap_t map)
{
	if (map == NULL) {
		return NULL;
	}
	if (((bus_iocap_dmamap_t)map)->magic == BUS_IOCAP_DMAMAP_MAGIC) {
		return (bus_iocap_dmamap_t)map;
	}
	return NULL;
}

// Takes the DMA mapping, a pointer to a physical segment (which MUST have been generated by that dmamap),
// and the permissions for that physical segment and generates an IOCap
// using the secret key assigned to the dmamap.
//
// TODO HOLDOVER: If the map doesn't have a key assigned, assigns the key for you. DO NOT CALL FROM MULTIPLE THREADS AT ONCE.
// THIS ALSO TAKES A LOCK SO IS BLOCKING
//
// Returns 0 if successful,
// EPERM if the map is not usable for minting OR if bus_dmamap_sync has not yet been called and iocap_enabled_tag_assign_key fails,
// and EDOM if ccap2024_11_init_cavs_exact fails.
int
bus_dmamap_mint_iocap(bus_iocap_dmamap_t map, bus_dma_segment_t *segment,
		CCapPerms perms, struct iocap *out)
{
	if (!bus_dmamap_can_mint_iocap((bus_dmamap_t)map)) {
		return EPERM;
	}
	switch (map->state) {
	case iocap_dmamap_key_selected:
		break;
	case iocap_dmamap_loaded: {
		// key not selected!
		// TODO FOR NOW DO THIS BUT ITS HACKY. AND ALSO THREAD UNSAFE
		int tag_assign_status = iocap_enabled_tag_assign_key(map->tag, map);
		if (tag_assign_status != 0) {
			return tag_assign_status;
		}
		KASSERT(map->state == iocap_dmamap_key_selected, ("iocap_enabled_tag_assign_key succeeded but didn't transition state"));
		break;
	}
	default:
		// The key_id stored in the map may not be valid
		return EPERM;
	}

	uint8_t key_id;
	CCapU128 *key;
	CCapResult res;

	// While the mapping is in the key_selected state, we assume the state of the indicated key is valid.
	// We do not need to take a lock on the key.

	key_id = map->tag->key_suballoc.key_datas[map->nth_key_of_tag].key_id;
	key = &map->tag->key_suballoc.key_datas[map->nth_key_of_tag].key_data;
	res = CCapResult_CatastrophicFailure;

	if (key != NULL) {
		res = ccap2024_11_init_cavs_exact(
			&out->cap,
			key,
			segment->ds_addr,
			segment->ds_len,
			key_id,
			perms);
	}

	if (res != CCapResult_Success) {
		device_printf(map->tag->iocap_keymngr,
				"failed to mint iocap for %lx..%lx with key #%d (%p) perms %s: %s\n",
				segment->ds_addr, segment->ds_len, key_id, key,
				ccap_perms_str(perms), ccap_result_str(res));
		return (key == NULL) ? EPERM : EDOM;
	}

	return 0;
}

// Takes the DMA mapping, a pointer to a physical segment (which MUST have been generated by that dmamap),
// the virtio flags(cannot have both), and the virtio next field, and generates an IOCap
// encoding all that information.
// This uses ccap2024_11_init_virtio_cavs_exact and thus combines some flags and the next field into the secret_key_id
// storage on the IOCap.
//
// TODO HOLDOVER: If the map doesn't have a key assigned, assigns the key for you. DO NOT CALL FROM MULTIPLE THREADS AT ONCE.
// THIS ALSO TAKES A LOCK SO IS BLOCKING
//
// Returns 0 if successful,
// EPERM if the map is not usable for minting OR if bus_dmamap_sync has not yet been called and iocap_enabled_tag_assign_key fails,
// and EDOM if ccap2024_11_init_virtio_cavs_exact fails or if the segment length >4GiB
int
bus_dmamap_mint_virtio_iocap(bus_iocap_dmamap_t map, bus_dma_segment_t *segment,
		uint16_t flags, uint16_t next, struct iocap *out)
{
	if (!bus_dmamap_can_mint_iocap((bus_dmamap_t)map)) {
		return EPERM;
	}
	switch (map->state) {
	case iocap_dmamap_key_selected:
		break;
	case iocap_dmamap_loaded: {
		// key not selected!
		// TODO FOR NOW DO THIS BUT ITS HACKY. AND ALSO THREAD UNSAFE
		int tag_assign_status = iocap_enabled_tag_assign_key(map->tag, map);
		if (tag_assign_status != 0) {
			return tag_assign_status;
		}
		KASSERT(map->state == iocap_dmamap_key_selected, ("iocap_enabled_tag_assign_key succeeded but didn't transition state"));
		break;
	}
	default:
		// The key_id stored in the map may not be valid
		return EPERM;
	}

	if (segment->ds_len >> 32) {
		return EDOM;
	}

	uint8_t key_id;
	CCapU128 *key;
	CCapResult res;
	CCapNativeVirtqDesc desc;

	// While the mapping is in the key_selected state, we assume the state of the indicated key is valid.
	// We do not need to take a lock on the key.

	key_id = map->tag->key_suballoc.key_datas[map->nth_key_of_tag].key_id;
	key = &map->tag->key_suballoc.key_datas[map->nth_key_of_tag].key_data;
	res = CCapResult_CatastrophicFailure;

	if (key != NULL) {
		desc = (CCapNativeVirtqDesc) {
			.addr = segment->ds_addr,
			.len = segment->ds_len,
			.flags = flags,
			.next = next
		};

		res = ccap2024_11_init_virtio_cavs_exact(
				&out->cap,
				key,
				&desc,
				key_id);

		// device_printf(map->tag->iocap_keymngr, "minting (data %02x%02x%02x%02x) (sig %02x%02x%02x%02x) for key_id %d (%02x%02x%02x%02x)\n",
		// 	out->cap.data[3],
		// 	out->cap.data[2],
		// 	out->cap.data[1],
		// 	out->cap.data[0],
		// 	out->cap.signature[3],
		// 	out->cap.signature[2],
		// 	out->cap.signature[1],
		// 	out->cap.signature[0],
		// 	key_id,
		// 	(*key)[3],
		// 	(*key)[2],
		// 	(*key)[1],
		// 	(*key)[0]
		// );
	}

	if (res != CCapResult_Success) {
		device_printf(map->tag->iocap_keymngr,
				"failed to mint virtio iocap for %lx..%lx with key #%d (%p) flags %x next %d: %s\n",
				segment->ds_addr, segment->ds_len, key_id, key,
				flags, next, ccap_result_str(res));
		return (key == NULL) ? EPERM : EDOM;
	}

	return 0;
}
