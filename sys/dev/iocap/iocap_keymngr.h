#ifndef IOCAP_KEYMNGR_H
#define IOCAP_KEYMNGR_H

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
//    but additionally includes all
// Type 1 will be returned by the iocap_bus by default. IOCap-able drivers can call bus_dma_tag_iocap_refinable() on any tag
// to see if it is Type 1. If it is type 1, then they can call bus_dma_tag_refine_to_iocap_group() on it with extra
// parameters such as revocation properties to refine it to a type 2 tag, and then create DMA mappings from that type 2
// tag to mint IOCaps. (TODO they need a function to extract the IOCap from an IOCap-enabled-mapping)

// BUT this is only compatible with those busdma_bounce systems - I don't know when the other kinds like busdma_machdep
// are used, and in practice it will only be tested on CHERI-RISC-V.

// Return 1 if the given bus_dma_tag_t is an IOCap-able tag - i.e. whether you can call bus_dma_tag_refine_to_iocap_group on it.
// Otherwise returns 0.
// TODO: on other systems this should be hardcoded to return 0.
int bus_dma_tag_iocap_refinable(bus_dma_tag_t tag);

struct iocap_keymngr_revocation_params {};

// Take a generic IOCap-able tag and refine it to a Type 2 i.e. IOCap Key Group Tag.
// Returns 0 if successful, otherwise returns an error code (TODO what error code)
int bus_dma_tag_refine_to_iocap_group(
    bus_dma_tag_t tag,
    struct iocap_keymngr_revocation_params params,
    bus_dma_tag_t* out
);

#endif