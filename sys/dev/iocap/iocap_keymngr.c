// Probing and setup based on uart_bus_fdt.c

#include <sys/types.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/iocap/iocap_keymngr.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "dev/fdt/simplebus.h"

static int iocap_keymngr_probe(device_t);
static int iocap_keymngr_attach(device_t);
static int iocap_keymngr_detach(device_t);
static void iocap_keymngr_dbg_perfcounters(device_t);

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
};

static device_method_t iocap_keymngr_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		iocap_keymngr_probe),
	DEVMETHOD(device_attach,	iocap_keymngr_attach),
	DEVMETHOD(device_detach,	iocap_keymngr_detach),
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
