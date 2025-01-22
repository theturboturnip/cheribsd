// Probing and setup based on uart_bus_fdt.c

#include <sys/types.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/bus_dma.h>
#include <machine/bus_dma_impl.h>
#include <machine/resource.h>

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
static_assert(offsetof(struct bus_dma_tag_common, impl) == 0, "struct bus_dma_tag_common must start with an impl vtable");

struct bus_dma_iocap_refinable_tag {
	struct bus_dma_tag_common common;
	device_t iocap_keymngr;
	// Unfortunate reality: we need to malloc() a pointer to (struct bus_dma_tag) by getting a DMA tag from our parent,
	// we can't redirect calls here.
	bus_dma_tag_t wrapper_ptr_to_base_tag;
	// TODO IS THIS NECESSARY
	// // Set to true if there are any non-iocap mappings created through this tag.
	// // If so, we don't refine it(?)
	// int has_non_iocap_mappings;
};

struct bus_dma_iocap_enabled_tag {
	struct bus_dma_tag_common common;
	device_t iocap_keymngr;
	bus_dma_tag_t wrapper_ptr_to_base_tag;
	// TODO
};

struct bus_dma_impl bus_dma_iocap_refinable_tag_impl;

static int iocap_keymngr_probe(device_t);
static int iocap_keymngr_attach(device_t);
static int iocap_keymngr_detach(device_t);
static void iocap_keymngr_dbg_perfcounters(device_t);

static bus_get_dma_tag_t iocap_keymngr_get_dma_tag;

struct iocap_keymngr_softc {
	struct simplebus_softc base;

	device_t	dev;

	// see sys/dev/uart/uart.h
	bus_space_tag_t bst;
	bus_space_handle_t bsh;

	struct resource	*sc_rres;	/* Register resource. */
	int		sc_rrid;
	int		sc_rtype;	/* SYS_RES_{IOPORT|MEMORY}. */

	// bus_dma_tag_t	sc_dmat;
	// struct mtx		 iocap_keymngr_mtx;

	struct bus_dma_iocap_refinable_tag base_refinable_tag;
};

static device_method_t iocap_keymngr_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		iocap_keymngr_probe),
	DEVMETHOD(device_attach,	iocap_keymngr_attach),
	DEVMETHOD(device_detach,	iocap_keymngr_detach),

	DEVMETHOD(bus_get_dma_tag,	iocap_keymngr_get_dma_tag),

	{ 0, 0 }
};

DEFINE_CLASS_1(iocap_keymngr, iocap_keymngr_driver, iocap_keymngr_methods,
	sizeof(struct iocap_keymngr_softc), simplebus_driver);

EARLY_DRIVER_MODULE(iocap_keymngr, ofwbus, iocap_keymngr_driver, 0, 0, BUS_PASS_BUS);
EARLY_DRIVER_MODULE(iocap_keymngr, simplebus, iocap_keymngr_driver, 0, 0,
	BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE);

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
 	device_printf(dev, "w00t attached to iocap!!! parent: %p\n", device_get_parent(dev));

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
	sc->sc_rres = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->sc_rrid,
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
	sc->base_refinable_tag.wrapper_ptr_to_base_tag = bus_get_dma_tag(dev);

	iocap_keymngr_dbg_perfcounters(dev);

	// TODO setup lock

	// TODO use
	// // From virtio_blk_iocap.c
	// error = bus_dma_tag_create(
	//     bus_get_dma_tag(dev),			/* parent */
	//     0,						/* alignment*/
	//     0,						/* boundary */
	//     BUS_SPACE_MAXADDR,				/* lowaddr */
	//     BUS_SPACE_MAXADDR,				/* highaddr */
	//     NULL, NULL,					/* filter, filterarg */
	//     maxphys,					/* max request size */
	//     sc->vtblk_iocap_max_nsegs - VTBLK_MIN_SEGMENTS,	/* max # segments */
	//     maxphys,					/* maxsegsize */
	//     0,						/* flags */
	//     busdma_lock_mutex,				/* lockfunc */
	//     &sc->iocap_keymngr_mtx,				/* lockarg */
	//     &sc->sc_dmat
	// );
	// if (error) {
	// 	device_printf(dev, "cannot create bus dma tag\n");
	// 	goto fail;
	// }

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

	// TODO
	// 	VTBLK_LOCK(sc);
	// sc->vtblk_iocap_flags |= VTBLK_FLAG_DETACH;
	// if (device_is_attached(dev))
	// 	vtblk_iocap_stop(sc);
	// VTBLK_UNLOCK(sc);

	if (sc->sc_rres != NULL)
		bus_release_resource(dev, sc->sc_rtype, sc->sc_rrid, sc->sc_rres);

	// TODO destroy lock

	return err;
}

static void iocap_keymngr_dbg_perfcounters(device_t dev)
{
	struct iocap_keymngr_softc *sc;

	sc = device_get_softc(dev);

	// device_printf(dev, "iocap_keymngr_dbg_perfcounters bst %p bsh %zu bsz %zu\n", sc->bst, (size_t)sc->bsh, (size_t)sc->bsz);
	uint64_t good_read  = bus_space_read_8(sc->bst, sc->bsh, 0x1000);
	// device_printf(dev, "iocap_keymngr_dbg_perfcounters good_read\n");
	uint64_t bad_read   = bus_space_read_8(sc->bst, sc->bsh, 0x1008);
	// device_printf(dev, "iocap_keymngr_dbg_perfcounters bad_read\n");
	uint64_t good_write = bus_space_read_8(sc->bst, sc->bsh, 0x1010);
	// device_printf(dev, "iocap_keymngr_dbg_perfcounters good_write\n");
	uint64_t bad_write  = bus_space_read_8(sc->bst, sc->bsh, 0x1018);
	device_printf(dev, "perf counters: %ld %ld %ld %ld\n", good_read, bad_read, good_write, bad_write);
}

static int refinable_tag_create(bus_dma_tag_t parent,
		bus_size_t alignment, bus_addr_t boundary, bus_addr_t lowaddr,
		bus_addr_t highaddr, bus_size_t maxsize, int nsegments,
		bus_size_t maxsegsz, int flags, bus_dma_lock_t *lockfunc,
		void *lockfuncarg, bus_dma_tag_t *dmat) {
	// TODO return a refinable tag not the default one?
	struct bus_dma_iocap_refinable_tag* refine_parent = (struct bus_dma_iocap_refinable_tag*)parent;
	// struct bus_dma_impl* parent_base_impl = ((struct bus_dma_tag_common*)refine_parent->wrapper_ptr_to_base_tag)->impl;
	return bus_dma_tag_create(
		refine_parent->wrapper_ptr_to_base_tag, alignment, boundary, lowaddr, highaddr, NULL, NULL, maxsize, nsegments, maxsegsz, flags, lockfunc, lockfuncarg, dmat
	);
}
static int refinable_tag_destroy(bus_dma_tag_t dmat) {
	struct bus_dma_iocap_refinable_tag* refine_dmat = (struct bus_dma_iocap_refinable_tag*)dmat;
	return bus_dma_tag_destroy(refine_dmat->wrapper_ptr_to_base_tag);
}
static int refinable_map_create(bus_dma_tag_t dmat, int flags, bus_dmamap_t *mapp) {
	struct bus_dma_iocap_refinable_tag* refine_dmat = (struct bus_dma_iocap_refinable_tag*)dmat;
	return bus_dmamap_create(refine_dmat->wrapper_ptr_to_base_tag, flags, mapp);
}
static int refinable_map_destroy(bus_dma_tag_t dmat, bus_dmamap_t map) {
	struct bus_dma_iocap_refinable_tag* refine_dmat = (struct bus_dma_iocap_refinable_tag*)dmat;
	return bus_dmamap_destroy(refine_dmat->wrapper_ptr_to_base_tag, map);
}
static int refinable_mem_alloc(bus_dma_tag_t dmat, void** vaddr, int flags,
bus_dmamap_t *mapp) {
	struct bus_dma_iocap_refinable_tag* refine_dmat = (struct bus_dma_iocap_refinable_tag*)dmat;
	struct bus_dma_impl* base_impl = ((struct bus_dma_tag_common*)refine_dmat->wrapper_ptr_to_base_tag)->impl;
	return base_impl->mem_alloc(refine_dmat->wrapper_ptr_to_base_tag, vaddr, flags, mapp);
}
static void refinable_mem_free(bus_dma_tag_t dmat, void *vaddr, bus_dmamap_t map) {
	struct bus_dma_iocap_refinable_tag* refine_dmat = (struct bus_dma_iocap_refinable_tag*)dmat;
	struct bus_dma_impl* base_impl = ((struct bus_dma_tag_common*)refine_dmat->wrapper_ptr_to_base_tag)->impl;
	return base_impl->mem_free(refine_dmat->wrapper_ptr_to_base_tag, vaddr, map);
}
static int refinable_load_ma(bus_dma_tag_t dmat, bus_dmamap_t map,
	struct vm_page **ma, bus_size_t tlen, int ma_offs, int flags,
	bus_dma_segment_t *segs, int *segp) {
	struct bus_dma_iocap_refinable_tag* refine_dmat = (struct bus_dma_iocap_refinable_tag*)dmat;
	return _bus_dmamap_load_ma(refine_dmat->wrapper_ptr_to_base_tag, map, ma, tlen, ma_offs, flags, segs, segp);
}
static int refinable_load_phys(bus_dma_tag_t dmat, bus_dmamap_t map,
	vm_paddr_t buf, bus_size_t buflen, int flags,
	bus_dma_segment_t *segs, int *segp) {
	struct bus_dma_iocap_refinable_tag* refine_dmat = (struct bus_dma_iocap_refinable_tag*)dmat;
	return _bus_dmamap_load_phys(refine_dmat->wrapper_ptr_to_base_tag, map, buf, buflen, flags, segs, segp);
}
static int refinable_load_buffer(bus_dma_tag_t dmat, bus_dmamap_t map,
	void *buf, bus_size_t buflen, struct pmap *pmap, int flags,
	bus_dma_segment_t *segs, int *segp) {
	struct bus_dma_iocap_refinable_tag* refine_dmat = (struct bus_dma_iocap_refinable_tag*)dmat;
	return _bus_dmamap_load_buffer(refine_dmat->wrapper_ptr_to_base_tag, map, buf, buflen, pmap, flags, segs, segp);
}
static void refinable_map_waitok(bus_dma_tag_t dmat, bus_dmamap_t map,
	struct memdesc *mem, bus_dmamap_callback_t *callback,
	void *callback_arg) {
	struct bus_dma_iocap_refinable_tag* refine_dmat = (struct bus_dma_iocap_refinable_tag*)dmat;
	_bus_dmamap_waitok(refine_dmat->wrapper_ptr_to_base_tag, map, mem, callback, callback_arg);
}
static bus_dma_segment_t *refinable_map_complete(bus_dma_tag_t dmat, bus_dmamap_t map,
bus_dma_segment_t *segs, int nsegs, int error) {
	struct bus_dma_iocap_refinable_tag* refine_dmat = (struct bus_dma_iocap_refinable_tag*)dmat;
	return _bus_dmamap_complete(refine_dmat->wrapper_ptr_to_base_tag, map, segs, nsegs, error);
}
static void refinable_map_unload(bus_dma_tag_t dmat, bus_dmamap_t map) {
	struct bus_dma_iocap_refinable_tag* refine_dmat = (struct bus_dma_iocap_refinable_tag*)dmat;
	bus_dmamap_unload(refine_dmat->wrapper_ptr_to_base_tag, map);
}
static void refinable_map_sync(bus_dma_tag_t dmat, bus_dmamap_t map,
bus_dmasync_op_t op) {
	struct bus_dma_iocap_refinable_tag* refine_dmat = (struct bus_dma_iocap_refinable_tag*)dmat;
	bus_dmamap_sync(refine_dmat->wrapper_ptr_to_base_tag, map, op);
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

/*
struct bus_dma_impl bus_dma_iocap_enabled_tag_impl = {
	.tag_create = todo,
	.tag_destroy = todo,
	.map_create = todo,
	.map_destroy = todo,
	.mem_alloc = todo,
	.mem_free = todo,
	.load_phys = todo,
	.load_buffer = todo,
	.load_ma = todo,
	.map_waitok = todo,
	.map_complete = todo,
	.map_unload = todo,
	.map_sync = todo
};
*/

static bus_dma_tag_t iocap_keymngr_get_dma_tag(device_t dev, device_t child) {
	struct iocap_keymngr_softc *sc;

	sc = device_get_softc(dev);

	device_printf(dev, "giving a refinable dmatag\n");

	return (bus_dma_tag_t)&sc->base_refinable_tag;

	// bus_dma_tag_t parent_tag = bus_get_dma_tag(dev);
	// struct bus_dma_iocap_refinable_tag* tag = malloc(sizeof(struct bus_dma_iocap_refinable_tag), M_DEVBUF, M_ZERO | M_NOWAIT);
	// tag->common.impl = &bus_dma_iocap_refinable_tag_impl;
	// // don't initialize the rest of common, we don't use it. we just call direct into the parent_tag via the impl
	// tag->iocap_keymngr = dev;
	// tag->wrapper_ptr_to_base_tag = parent_tag;
	//
	//
	// return tag;
}

int bus_dma_tag_iocap_refinable(bus_dma_tag_t tag) {
	if (((struct bus_dma_tag_common*)tag)->impl == &bus_dma_iocap_refinable_tag_impl) {
		return 1;
	}
	return 0;
}

int bus_dma_tag_refine_to_iocap_group(bus_dma_tag_t tag, struct iocap_keymngr_revocation_params params, bus_dma_tag_t* out) {
	if (!bus_dma_tag_iocap_refinable(tag)) {
		return EPERM;
	}

	// struct bus_dma_iocap_refinable_tag* refine_tag = (struct bus_dma_iocap_refinable_tag*)tag;
	return EPERM; // TODO
}
