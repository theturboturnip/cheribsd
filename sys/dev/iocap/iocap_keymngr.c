// Probing and setup baed on uart_bus_fdt.c

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

static int iocap_keymngr_probe(device_t);
static int iocap_keymngr_attach(device_t);
static int iocap_keymngr_detach(device_t);

struct iocap_keymngr_softc {
	device_t	dev;

	struct resource	*sc_rres;	/* Register resource. */
	int		sc_rrid;
	int		sc_rtype;	/* SYS_RES_{IOPORT|MEMORY}. */

	// bus_dma_tag_t	sc_dmat;
	// struct mtx		 iocap_keymngr_mtx;
};

static struct ofw_compat_data compat_data[] = {
	{ "sws35,iocap_keymngr",	1 },
	{ NULL, 0 }
};

static device_method_t iocap_keymngr_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		iocap_keymngr_probe),
	DEVMETHOD(device_attach,	iocap_keymngr_attach),
	DEVMETHOD(device_detach,	iocap_keymngr_detach),
	{ 0, 0 }
};

static driver_t iocap_keymngr_driver = {
	"iocap_keymngr",
	iocap_keymngr_methods,
	sizeof(struct iocap_keymngr_softc),
};

static int
iocap_keymngr_probe(device_t dev)
{
	const struct ofw_compat_data *ocd;

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	ocd = ofw_bus_search_compatible(dev, compat_data);
	if (ocd->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "IOCap Key Manager");
	return (BUS_PROBE_DEFAULT);
}

static int
iocap_keymngr_attach(device_t dev)
{
 	device_printf(dev, "w00t attached to iocap!!!\n");

	struct iocap_keymngr_softc *sc;
	int error;

	sc = device_get_softc(dev);
	error = 0;

	// From uart_core.c:uart_bus_probe
	/*
	 * Allocate the register resource. We assume that all UARTs have
	 * a single register window in either I/O port space or memory
	 * mapped I/O space. Any UART that needs multiple windows will
	 * consequently not be supported by this driver as-is. We try I/O
	 * port space first because that's the common case.
	 */
	sc->sc_rrid = 0;
	sc->sc_rtype = SYS_RES_IOPORT;
	sc->sc_rres = bus_alloc_resource_any(dev, sc->sc_rtype, &sc->sc_rrid,
	    RF_ACTIVE);
	if (sc->sc_rres == NULL) {
		sc->sc_rrid = 0;
		sc->sc_rtype = SYS_RES_MEMORY;
		sc->sc_rres = bus_alloc_resource_any(dev, sc->sc_rtype,
		    &sc->sc_rrid, RF_ACTIVE);
		if (sc->sc_rres == NULL) {
			device_printf(dev, "unable to allocate memory\n");
			error = 1; // TODO error code
			goto fail;
		}
	}

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
	if (error)
		iocap_keymngr_detach(dev);

	return (error);
}

static int
iocap_keymngr_detach(device_t dev)
{
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

	return 0;
}

// 	// node = ofw_bus_get_node(dev);
//

//
//
// 	return BUS_PROBE_DEFAULT;
// }

// TODO make this EARLY_?
DRIVER_MODULE(iocap_keymngr, simplebus, iocap_keymngr_driver, 0, 0);
DRIVER_MODULE(iocap_keymngr, ofwbus, iocap_keymngr_driver, 0, 0);
