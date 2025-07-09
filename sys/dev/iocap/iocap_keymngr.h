#ifndef IOCAP_KEYMNGR_H
#define IOCAP_KEYMNGR_H

#include <sys/bus_dma.h>
#include <dev/iocap/iocap.h>

// ======= IOCap DMA Tags =======

// The IOCap system slightly misuses the DMA tag system.
// bus_dma_tag_t is always defined as a typedef struct bus_dma_tag	*bus_dma_tag_t,
// described as a "machine-dependent opaque type" (see _bus_dma.h).
// Note this is machine-dependent, but not bus-dependent.
// Indeed, the structure bus_dma_tag is only defined once per machine(ish), each time in a single C translation unit
// - sys/arm/arm/busdma_machdep.c
// - stand/kshim/bsd_kernel.h
// - sys/powerpc/powerpc/busdma_machdep.c
// - sys/arm64/arm64/busdma_bounce.c
// - sys/riscv/riscv/busdma_bounce.c
// - sys/x86/x86/busdma_bounce.c
// In the busdma_bounce cases, the tag structures all start with a `struct bus_dma_tag_common` field,
// which is defined in sys/{riscv,arm64,x86}/include/bus_dma_impl.h with a first field `struct bus_dma_impl *impl`.
// This impl struct is a vtable (struct-of-function-pointers) defined in the same file.
// The same C translation unit which defines struct bus_dma_tag also defines a global `struct bus_dma_impl` pointing to
// static functions exclusive to that translation unit.
// These functions include the functions that create DMA mappings from DMA tags, and the canonical definition of
// bus_dma_tag_create (which is declared in sys/bus_dma.h, but defined in sys/<arch>/busdma_machdep.c) hooks the DMA tag `*impl`
// pointer to that global struct if the parent tag is NULL.
// The final step is sys/{riscv,arm64,x86}/include/bus_dma.h, which then defines common functions such as
// bus_dmamap_create to cast the `struct bus_dma_tag *` to `struct bus_dma_tag_common *` (which is legal because it is the first field)
// and then call the functions in the pointed-to `bus_dma_impl` vtable.

// To evaluate the usability of IOCaps in practice, I'd like to stick to the default OS APIs as much as possible,
// including overriding those functions.
// This is made possible by having a bus-device override the bus_get_dma_tag() method in its class, and returning
// a different (but still compatible) pointer - i.e. a pointer to a structure where the first field is a bus_dma_tag_common,
// which contains a pointer to a vtable which I control.
// The usual practice for creating DMA tags is to call bus_dma_tag_create() (defined in sys/<arch>/busdma_machdep.c)
// with the first parameter being bus_get_dma_tag(dev) (which I can override when I write the driver for dev).
// This *should* (crossing fingers) allow me to make a DMA tag struct with different fields to the bounce-buffer code,
// without changing anything in the bounce-buffer code, without breaking anything in the bounce-buffer code.
// In fact, there will be two kinds of IOCap DMA tag struct:
// 1. an IOCap-able but not IOCap-enabled shim around the full (struct bus_dma_tag) defined by the bounce buffer.
//      - non-IOCap-aware device drivers can use this as a basic tag
//      - once you know the properties a group of IOCaps will have (e.g. key revocation strategy) you can refine
//        this tag into Type 2:
// 2. an IOCap-enabled tag structure which (probably) still has all the data from the bounce buffer
//    but additionally reserves N key ids within the IOCap key manager for generating IOCaps.
//    These key IDs are then passed down to generated DMA mappings (on a many:1 mapping:keyID basis) and can be used to mint IOCaps.
// Type 1 will be returned by the iocap_bus by default. IOCap-able drivers can call bus_dma_tag_iocap_refinable() on any tag
// to see if it is Type 1. If it is type 1, then they can call bus_dma_tag_refine_to_iocap_group() on it with extra
// parameters such as revocation properties to refine it to a type 2 tag, and then create DMA mappings from that type 2
// tag to mint IOCaps.

// BUT this is only compatible with those busdma_bounce systems - I don't know when the other kinds like busdma_machdep
// are used, and in practice it will only be tested on CHERI-RISC-V.

// ======= IOCap DMA Mappings =======

// After you have a Type 2 IOCap-enabled DMA tag, you can use the standard APIs to generate dmamap objects.
// These defer the mapping to the underlying DMA tag (i.e. the bounce buffer), and adhere to all constraints from that tag,
// but can be passed into bus_dmamap_mint{,_virtio}_iocap to create IOCaps using that secret key.
// Originally I planned to include IOCaps inside the bus_dma_segment_t objects returned by the dmamap _load() functions,
// but we do not have all the information we need at that point. The _load functions do not include the read/write permissions
// or the virtio-specific extra flags that ccap2024_11_init_virtio_cavs_exact stuffs into the key ID slot.
// Therefore we have a set of extra functions that take (dmamap, dma_segment, metadata) to mint IOCaps on-demand.
// TODO how do handle epoch-based revocation in this API? particularly selecting what epoch you allocate something under

// Forward-declare all types for extra safety. Drivers that rely on iocap behaviour can hold these more specific pointers
struct bus_dma_iocap_refinable_tag;
typedef struct bus_dma_iocap_refinable_tag *bus_dma_iocap_refinable_tag_t;

struct bus_dma_iocap_enabled_tag;
typedef struct bus_dma_iocap_enabled_tag *bus_dma_iocap_enabled_tag_t;

struct bus_iocap_dmamap;
typedef struct bus_iocap_dmamap *bus_iocap_dmamap_t;

// Return the argument as a pointer-to-refinable-tag if the given bus_dma_tag_t is an IOCap-able tag - i.e. whether you can call bus_dma_tag_refine_to_iocap_group on it.
// Otherwise returns NULL.
// TODO: on other systems this should be hardcoded to return NULL.
bus_dma_iocap_refinable_tag_t bus_dma_tag_iocap_refinable(bus_dma_tag_t tag);

enum iocap_keymngr_revocation_mode {
	// A single key, filled in with random data on demand once the first
	// mapping is bus_dmamap_sync()-d or the first IOCap is minted.
	// Requires n_keys = 1
	iocap_revoke_when_no_mappings_unsafe,
	// Pool of four keys, used with epochs.
	// Not guaranteed to be round-robin - consider the case where
	// the entire queue is backed up, and finally one epoch opens - no reason
	// not to use it!
	// TODO how to handle timeouts
	iocap_rolling_epoch_x4,
};

struct iocap_keymngr_revocation_params {
	enum iocap_keymngr_revocation_mode   mode;
	union {
		struct {
			// The maximum number of mappings used in an epoch before
			// rolling over to the next one.
			// Sets the maximum number of concurrent mappings as
			// (num epochs * max_num_mappings_per_epoch)
			// If 0, not used. max_bytes_mapped_per_epoch must
			// be used instead.
			uint64_t max_num_mappings_per_epoch;
			// // The maximum number of bytes used in an epoch before
			// // rolling over to the next one.
			// // Sets the maximum number of concurrent bytes mapped as
			// // (num epochs * max_bytes_mapped_per_epoch)
			// // If 0, not used. max_num_mappings_per_epoch must
			// // be used instead.
			// uint64_t max_bytes_mapped_per_epoch;
			// TODO max lifetime
		} rolling_epoch;
	} params;
};

// Take a generic IOCap-able tag and refine it to a Type 2 i.e. IOCap Key Group Tag.
// Returns 0 if successful, otherwise returns an error code (TODO what error code)
int bus_dma_tag_refine_to_iocap_group(
    bus_dma_iocap_refinable_tag_t tag,
    struct iocap_keymngr_revocation_params params,
    bus_dma_iocap_enabled_tag_t* out
)  __attribute__((warn_unused_result));

// Return the argument as a pointer-to-iocap-dmamap if the given bus_dmamap_t is an IOCap-able map - i.e. whether you can call bus_dmamap_mint_iocap
// or bus_dmamap_mint_virtio_iocap on it.
// Otherwise returns NULL.
bus_iocap_dmamap_t bus_dmamap_can_mint_iocap(bus_dmamap_t map);

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
int bus_dmamap_mint_iocap(bus_iocap_dmamap_t map, bus_dma_segment_t* segment, CCapPerms perms, struct iocap* out)  __attribute__((warn_unused_result));

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
int bus_dmamap_mint_virtio_iocap(bus_iocap_dmamap_t map, bus_dma_segment_t* segment, uint16_t flags, uint16_t next, struct iocap* out)  __attribute__((warn_unused_result));

// The callback for bus_dmamap_unload2, taking two caller-defined arguments
typedef void (*iocap_keymngr_bus_dmamap_unload2_cb)(void* arg1, void* arg2);

// Equivalent of bus_dmamap_unload that calls a callback once the relevant memory is inaccessible.
// That memory should not be used for any other purpose, unless that purpose is access by the same device,
// until the callback is called.
// The callback may be called immediately, and should not call any functions related to IOCaps
// as locks may be held.
void iocap_keymngr_bus_dmamap_unload2(bus_dma_iocap_enabled_tag_t tag,
	bus_iocap_dmamap_t map, iocap_keymngr_bus_dmamap_unload2_cb on_unmapped,
	void* arg1, void* arg2);

#endif