/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2011, Bryan Venteicher <bryanv@FreeBSD.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice unmodified, this list of conditions, and the following
 *    disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Driver for VirtIO block devices. */

// ReSharper disable CppDFAUnreachableFunctionCall
// ReSharper disable CppParameterMayBeConstPtrOrRef
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/bio.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/msan.h>
#include <sys/sglist.h>
#include <sys/sysctl.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/queue.h>

#include <geom/geom.h>
#include <geom/geom_disk.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <sys/bus.h>
#include <sys/rman.h>

#include <dev/iocap/iocap_keymngr.h>
#include <dev/virtio/virtio.h>
#include <dev/virtio/virtqueue_iocap.h>
#include <dev/virtio/block/virtio_blk.h>

#include "virtio_if.h"

struct vtblk_iocap_request {
	struct vtblk_iocap_softc		*vbr_sc;
	union {
		bus_dmamap_t				vbr_mapp;
		bus_iocap_dmamap_t			vbr_iocap_mapp;
	};

	/* Fields after this point are zeroed for each request. */
	struct virtio_blk_outhdr	 vbr_hdr;
	struct bio			*vbr_bp;
	uint8_t				 vbr_ack;
	uint8_t				 vbr_requeue_on_error;
	uint8_t				 vbr_busdma_wait;
	/* TODO set to 1 if it has been dequeued and is awaiting unmapping? */
	uint8_t				 vbr_quarantined;
	int				 vbr_error;
	TAILQ_ENTRY(vtblk_iocap_request)	 vbr_link;
};

enum vtblk_iocap_cache_mode {
	VTBLK_CACHE_WRITETHROUGH,
	VTBLK_CACHE_WRITEBACK,
	VTBLK_CACHE_MAX
};

struct vtblk_iocap_softc {
	device_t		 vtblk_iocap_dev;
	struct mtx		 vtblk_iocap_mtx;
	uint64_t		 vtblk_iocap_features;
	uint32_t		 vtblk_iocap_flags;
#define VTBLK_FLAG_INDIRECT	0x0001
#define VTBLK_FLAG_DETACH	0x0002
#define VTBLK_FLAG_SUSPEND	0x0004
#define VTBLK_FLAG_BARRIER	0x0008
#define VTBLK_FLAG_WCE_CONFIG	0x0010
#define VTBLK_FLAG_BUSDMA_WAIT	0x0020
#define VTBLK_FLAG_BUSDMA_ALIGN	0x0040

	struct virtq_iocap	*vtblk_iocap_vq;
	struct sglist		*vtblk_iocap_sglist;
	bus_dma_iocap_enabled_tag_t		vtblk_iocap_queues_tag;
	bus_dma_iocap_enabled_tag_t		vtblk_iocap_request_tag;
	struct disk		*vtblk_iocap_disk;

	struct bio_queue_head	 vtblk_iocap_bioq;
	TAILQ_HEAD(, vtblk_iocap_request)
				 vtblk_iocap_req_free;
	TAILQ_HEAD(, vtblk_iocap_request)
				 vtblk_iocap_req_ready;
	struct vtblk_iocap_request	*vtblk_iocap_req_ordered;

	int			 vtblk_iocap_max_nsegs;
	int			 vtblk_iocap_request_count;
	enum vtblk_iocap_cache_mode	 vtblk_iocap_write_cache;

	struct bio_queue	 vtblk_iocap_dump_queue;
	struct vtblk_iocap_request	 vtblk_iocap_dump_request;
};

static struct virtio_feature_desc vtblk_iocap_feature_desc[] = {
	{ VIRTIO_BLK_F_BARRIER,		"HostBarrier"	},
	{ VIRTIO_BLK_F_SIZE_MAX,	"MaxSegSize"	},
	{ VIRTIO_BLK_F_SEG_MAX,		"MaxNumSegs"	},
	{ VIRTIO_BLK_F_GEOMETRY,	"DiskGeometry"	},
	{ VIRTIO_BLK_F_RO,		"ReadOnly"	},
	{ VIRTIO_BLK_F_BLK_SIZE,	"BlockSize"	},
	{ VIRTIO_BLK_F_SCSI,		"SCSICmds"	},
	{ VIRTIO_BLK_F_FLUSH,		"FlushCmd"	},
	{ VIRTIO_BLK_F_TOPOLOGY,	"Topology"	},
	{ VIRTIO_BLK_F_CONFIG_WCE,	"ConfigWCE"	},
	{ VIRTIO_BLK_F_MQ,		"Multiqueue"	},
	{ VIRTIO_BLK_F_DISCARD,		"Discard"	},
	{ VIRTIO_BLK_F_WRITE_ZEROES,	"WriteZeros"	},
	{ VIRTIO_F_IOCAP_QUEUE,	"IOCap"	},

	{ 0, NULL }
};

static int	vtblk_iocap_modevent(module_t, int, void *);

static int	vtblk_iocap_probe(device_t);
static int	vtblk_iocap_attach(device_t);
static int	vtblk_iocap_detach(device_t);
static int	vtblk_iocap_suspend(device_t);
static int	vtblk_iocap_resume(device_t);
static int	vtblk_iocap_shutdown(device_t);
static int	vtblk_iocap_attach_completed(device_t);
static int	vtblk_iocap_config_change(device_t);

static int	vtblk_iocap_open(struct disk *);
static int	vtblk_iocap_close(struct disk *);
static int	vtblk_iocap_ioctl(struct disk *, u_long, void *, int,
		    struct thread *);
static int	vtblk_iocap_dump(void *, void *, off_t, size_t);
static void	vtblk_iocap_strategy(struct bio *);

static int	vtblk_iocap_negotiate_features(struct vtblk_iocap_softc *);
static int	vtblk_iocap_setup_features(struct vtblk_iocap_softc *);
static int	vtblk_iocap_maximum_segments(struct vtblk_iocap_softc *,
		    struct virtio_blk_config *);
static int	vtblk_iocap_alloc_virtqueue(struct vtblk_iocap_softc *);
static void	vtblk_iocap_resize_disk(struct vtblk_iocap_softc *, uint64_t);
static void	vtblk_iocap_alloc_disk(struct vtblk_iocap_softc *,
		    struct virtio_blk_config *);
static void	vtblk_iocap_create_disk(struct vtblk_iocap_softc *);

static int	vtblk_iocap_request_prealloc(struct vtblk_iocap_softc *);
static void	vtblk_iocap_request_free(struct vtblk_iocap_softc *);
static struct vtblk_iocap_request *
		vtblk_iocap_request_dequeue(struct vtblk_iocap_softc *);
static void	vtblk_iocap_request_enqueue(struct vtblk_iocap_softc *,
		    struct vtblk_iocap_request *);
static struct vtblk_iocap_request *
		vtblk_iocap_request_next_ready(struct vtblk_iocap_softc *);
static void	vtblk_iocap_request_requeue_ready(struct vtblk_iocap_softc *,
		    struct vtblk_iocap_request *);
static struct vtblk_iocap_request *
		vtblk_iocap_request_next(struct vtblk_iocap_softc *);
static struct vtblk_iocap_request *
		vtblk_iocap_request_bio(struct vtblk_iocap_softc *);
static int	vtblk_iocap_request_execute(struct vtblk_iocap_request *, int);
static void	vtblk_iocap_request_execute_cb(void *,
		    bus_dma_segment_t *, int, int);
static int	vtblk_iocap_request_error(struct vtblk_iocap_request *);

static void	vtblk_iocap_queue_completed(struct vtblk_iocap_softc *,
		    struct bio_queue *);
static void	vtblk_iocap_done_completed(struct vtblk_iocap_softc *,
		    struct bio_queue *);
static void	vtblk_iocap_drain_vq(struct vtblk_iocap_softc *);
static void	vtblk_iocap_drain(struct vtblk_iocap_softc *);

static void	vtblk_iocap_startio(struct vtblk_iocap_softc *);
static void	vtblk_iocap_bio_done(struct vtblk_iocap_softc *, struct bio *, int);

static void	vtblk_iocap_read_config(struct vtblk_iocap_softc *,
		    struct virtio_blk_config *);
static void	vtblk_iocap_ident(struct vtblk_iocap_softc *);
static int	vtblk_iocap_poll_request(struct vtblk_iocap_softc *,
		    struct vtblk_iocap_request *);
static int	vtblk_iocap_quiesce(struct vtblk_iocap_softc *);
static void	vtblk_iocap_vq_intr(void *);
static void	vtblk_iocap_stop(struct vtblk_iocap_softc *);

static void	vtblk_iocap_dump_quiesce(struct vtblk_iocap_softc *);
static int	vtblk_iocap_dump_write(struct vtblk_iocap_softc *, void *, off_t, size_t);
static int	vtblk_iocap_dump_flush(struct vtblk_iocap_softc *);
static void	vtblk_iocap_dump_complete(struct vtblk_iocap_softc *);

static void	vtblk_iocap_set_write_cache(struct vtblk_iocap_softc *, int);
static int	vtblk_iocap_write_cache_enabled(struct vtblk_iocap_softc *sc,
		    struct virtio_blk_config *);
static int	vtblk_iocap_write_cache_sysctl(SYSCTL_HANDLER_ARGS);

static void	vtblk_iocap_setup_sysctl(struct vtblk_iocap_softc *);
static int	vtblk_iocap_tunable_int(struct vtblk_iocap_softc *, const char *, int);

#define vtblk_iocap_modern(_sc) (((_sc)->vtblk_iocap_features & VIRTIO_F_VERSION_1) != 0)
#define vtblk_iocap_htog16(_sc, _val)	virtio_htog16(vtblk_iocap_modern(_sc), _val)
#define vtblk_iocap_htog32(_sc, _val)	virtio_htog32(vtblk_iocap_modern(_sc), _val)
#define vtblk_iocap_htog64(_sc, _val)	virtio_htog64(vtblk_iocap_modern(_sc), _val)
#define vtblk_iocap_gtoh16(_sc, _val)	virtio_gtoh16(vtblk_iocap_modern(_sc), _val)
#define vtblk_iocap_gtoh32(_sc, _val)	virtio_gtoh32(vtblk_iocap_modern(_sc), _val)
#define vtblk_iocap_gtoh64(_sc, _val)	virtio_gtoh64(vtblk_iocap_modern(_sc), _val)

/* Tunables. */
static int vtblk_iocap_no_ident = 0;
TUNABLE_INT("hw.vtblk_iocap.no_ident", &vtblk_iocap_no_ident);
static int vtblk_iocap_writecache_mode = -1;
TUNABLE_INT("hw.vtblk_iocap.writecache_mode", &vtblk_iocap_writecache_mode);

#define VTBLK_COMMON_FEATURES \
    (VIRTIO_BLK_F_SIZE_MAX		| \
     VIRTIO_BLK_F_SEG_MAX		| \
     VIRTIO_BLK_F_GEOMETRY		| \
     VIRTIO_BLK_F_RO			| \
     VIRTIO_BLK_F_BLK_SIZE		| \
     VIRTIO_BLK_F_FLUSH			| \
     VIRTIO_BLK_F_TOPOLOGY		| \
     VIRTIO_BLK_F_CONFIG_WCE		| \
     VIRTIO_BLK_F_DISCARD		| \
     VIRTIO_RING_F_INDIRECT_DESC | \
     VIRTIO_F_IOCAP_QUEUE)

#define VTBLK_MODERN_FEATURES	(VTBLK_COMMON_FEATURES)
#define VTBLK_LEGACY_FEATURES	(VIRTIO_BLK_F_BARRIER | VTBLK_COMMON_FEATURES)

#define VTBLK_MTX(_sc)		&(_sc)->vtblk_iocap_mtx
#define VTBLK_LOCK_INIT(_sc, _name) \
				mtx_init(VTBLK_MTX((_sc)), (_name), \
				    "VirtIO Block Lock", MTX_DEF)
#define VTBLK_LOCK(_sc)		mtx_lock(VTBLK_MTX((_sc)))
#define VTBLK_UNLOCK(_sc)	mtx_unlock(VTBLK_MTX((_sc)))
#define VTBLK_LOCK_DESTROY(_sc)	mtx_destroy(VTBLK_MTX((_sc)))
#define VTBLK_LOCK_ASSERT(_sc)	mtx_assert(VTBLK_MTX((_sc)), MA_OWNED)
#define VTBLK_LOCK_ASSERT_NOTOWNED(_sc) \
				mtx_assert(VTBLK_MTX((_sc)), MA_NOTOWNED)

#define VTBLK_DISK_NAME		"vtbd-iocap"
#define VTBLK_QUIESCE_TIMEOUT	(30 * hz)
#define VTBLK_BSIZE		512

/*
 * Each block request uses at least two segments - one for the header
 * and one for the status.
 */
#define VTBLK_MIN_SEGMENTS	2

static device_method_t vtblk_iocap_methods[] = {
	/* Device methods. */
	DEVMETHOD(device_probe,		vtblk_iocap_probe),
	DEVMETHOD(device_attach,	vtblk_iocap_attach),
	DEVMETHOD(device_detach,	vtblk_iocap_detach),
	DEVMETHOD(device_suspend,	vtblk_iocap_suspend),
	DEVMETHOD(device_resume,	vtblk_iocap_resume),
	DEVMETHOD(device_shutdown,	vtblk_iocap_shutdown),

	/* VirtIO methods. */
	DEVMETHOD(virtio_attach_completed, vtblk_iocap_attach_completed),
	DEVMETHOD(virtio_config_change,	vtblk_iocap_config_change),

	DEVMETHOD_END
};

static driver_t vtblk_iocap_driver = {
	"vtblk_iocap",
	vtblk_iocap_methods,
	sizeof(struct vtblk_iocap_softc)
};

VIRTIO_DRIVER_MODULE(virtio_blk_iocap, vtblk_iocap_driver, vtblk_iocap_modevent, NULL);
MODULE_VERSION(virtio_blk_iocap, 1);
MODULE_DEPEND(virtio_blk_iocap, virtio, 1, 1, 1);

VIRTIO_SIMPLE_PNPINFO(virtio_blk_iocap, VIRTIO_ID_BLOCK, "VirtIO Block Adapter (IOCap)");

static int
vtblk_iocap_modevent(module_t mod, int type, void *unused)
{
	int error;

	error = 0;

	switch (type) {
	case MOD_LOAD:
	case MOD_QUIESCE:
	case MOD_UNLOAD:
	case MOD_SHUTDOWN:
		break;
	default:
		error = EOPNOTSUPP;
		break;
	}

	return (error);
}

static int
vtblk_iocap_probe(device_t dev)
{
	if (virtio_get_device_type(dev) != virtio_blk_iocap_match.device_type)
		return (ENXIO);
	// If IOCaps are not supported by the device, reject it. The no-IOCap-specific driver will pick it up.
	// TODO search the parent chain for a IOCap-capable bus. If it isn't present, drop to the IOCap driver.
	if (virtio_get_iocap_support(dev) == 0)
		return (ENXIO);
	if (bus_dma_tag_iocap_refinable(bus_get_dma_tag(dev)) == 0)
		return (ENXIO);
	device_set_desc(dev, virtio_blk_iocap_match.description);
	// TODO switch to BUS_PROBE_VENDOR or BUS_PROBE_SPECIFIC and remove the not-iocap logic from the virtio_blk driver
	return (BUS_PROBE_DEFAULT);
}

static int
vtblk_iocap_attach(device_t dev)
{
	struct vtblk_iocap_softc *sc;
	struct virtio_blk_config blkcfg;
	int error;
	bus_dma_tag_t intermediate_tag;
	bus_dma_iocap_refinable_tag_t	refinable_tag;


	sc = device_get_softc(dev);
	sc->vtblk_iocap_dev = dev;
	virtio_set_feature_desc(dev, vtblk_iocap_feature_desc);

	VTBLK_LOCK_INIT(sc, device_get_nameunit(dev));
	bioq_init(&sc->vtblk_iocap_bioq);
	TAILQ_INIT(&sc->vtblk_iocap_dump_queue);
	TAILQ_INIT(&sc->vtblk_iocap_req_free);
	TAILQ_INIT(&sc->vtblk_iocap_req_ready);

	vtblk_iocap_setup_sysctl(sc);

	error = vtblk_iocap_setup_features(sc);
	if (error) {
		device_printf(dev, "cannot setup features\n");
		goto fail;
	}

	vtblk_iocap_read_config(sc, &blkcfg);

	/*
	 * With the current sglist(9) implementation, it is not easy
	 * for us to support a maximum segment size as adjacent
	 * segments are coalesced. For now, just make sure it's larger
	 * than the maximum supported transfer size.
	 */
	if (virtio_with_feature(dev, VIRTIO_BLK_F_SIZE_MAX)) {
		if (blkcfg.size_max < maxphys) {
			error = ENOTSUP;
			device_printf(dev, "host requires unsupported "
			    "maximum segment size feature\n");
			goto fail;
		}
	}

	sc->vtblk_iocap_max_nsegs = vtblk_iocap_maximum_segments(sc, &blkcfg);
	if (sc->vtblk_iocap_max_nsegs <= VTBLK_MIN_SEGMENTS) {
		error = EINVAL;
		device_printf(dev, "fewer than minimum number of segments "
		    "allowed: %d\n", sc->vtblk_iocap_max_nsegs);
		goto fail;
	}

	sc->vtblk_iocap_sglist = sglist_alloc(sc->vtblk_iocap_max_nsegs, M_NOWAIT);
	if (sc->vtblk_iocap_sglist == NULL) {
		error = ENOMEM;
		device_printf(dev, "cannot allocate sglist\n");
		goto fail;
	}

	/*
	 * If vtblk_iocap_max_nsegs == VTBLK_MIN_SEGMENTS + 1, the device only
	 * supports a single data segment; in that case we need busdma to
	 * align to a page boundary so we can send a *contiguous* page size
	 * request to the host.
	 */
	if (sc->vtblk_iocap_max_nsegs == VTBLK_MIN_SEGMENTS + 1)
		sc->vtblk_iocap_flags |= VTBLK_FLAG_BUSDMA_ALIGN;
	error = bus_dma_tag_create(
	    bus_get_dma_tag(dev),			/* parent */
	    (sc->vtblk_iocap_flags & VTBLK_FLAG_BUSDMA_ALIGN) ? PAGE_SIZE : 1,
	    0,						/* boundary */
	    BUS_SPACE_MAXADDR,				/* lowaddr */
	    BUS_SPACE_MAXADDR,				/* highaddr */
	    NULL, NULL,					/* filter, filterarg */
	    maxphys,					/* max request size */
	    sc->vtblk_iocap_max_nsegs - VTBLK_MIN_SEGMENTS,	/* max # segments */
	    maxphys,					/* maxsegsize */
	    0,						/* flags */
	    busdma_lock_mutex,				/* lockfunc */
	    &sc->vtblk_iocap_mtx,				/* lockarg */
	    &intermediate_tag);
	if (error) {
		device_printf(dev, "cannot create bus dma tag\n");
		goto fail;
	}
	// Initialize vtblk_iocap_refinable_dmat and vtblk_dmat at the same time.
	// They're in a union so that we can reference as generic bus_dma_tag_t
	// when convenient
	refinable_tag = bus_dma_tag_iocap_refinable(intermediate_tag);
	if (refinable_tag == NULL) {
		error = ENOTSUP;
		device_printf(dev, "bus dma tag was not iocap-refinable\n");
		goto fail;
	}

	error = bus_dma_tag_refine_to_iocap_group(
		refinable_tag,
		(struct iocap_keymngr_revocation_params) {
			// This is actually fine, because we always unload all mappings together
			// => lifetime of any given mapping = lifetime of tag
			.mode = iocap_revoke_when_no_mappings_unsafe,
		},
		&sc->vtblk_iocap_queues_tag
	);
	if (error) {
		device_printf(dev, "cannot refine bus dma tag to iocap group for holding queues\n");
		goto fail;
	}

	error = bus_dma_tag_refine_to_iocap_group(
		refinable_tag,
		(struct iocap_keymngr_revocation_params) {
			.mode = iocap_rolling_epoch_x4,
			.params = {
				.rolling_epoch = {
					// TODO change this!
					.max_num_mappings_per_epoch = 1,
				}
			}
		},
		&sc->vtblk_iocap_request_tag
	);
	if (error) {
		device_printf(dev, "cannot refine bus dma tag to iocap group for holding requests for queue #0\n");
		goto fail;
	}

#ifdef __powerpc__
	/*
	 * Virtio uses physical addresses rather than bus addresses, so we
	 * need to ask busdma to skip the iommu physical->bus mapping.  At
	 * present, this is only a thing on the powerpc architectures.
	 */
	bus_dma_tag_set_iommu(sc->vtblk_iocap_dmat, NULL, NULL);
#endif

	error = vtblk_iocap_alloc_virtqueue(sc);
	if (error) {
		device_printf(dev, "cannot allocate virtqueue\n");
		goto fail;
	}

	error = vtblk_iocap_request_prealloc(sc);
	if (error) {
		device_printf(dev, "cannot preallocate requests\n");
		goto fail;
	}

	vtblk_iocap_alloc_disk(sc, &blkcfg);

	error = virtio_setup_intr(dev, INTR_TYPE_BIO | INTR_ENTROPY);
	if (error) {
		device_printf(dev, "cannot setup virtqueue interrupt\n");
		goto fail;
	}

	virtq_iocap_enable_intr(sc->vtblk_iocap_vq);

fail:
	if (error)
		vtblk_iocap_detach(dev);

	return (error);
}

static int
vtblk_iocap_detach(device_t dev)
{
	struct vtblk_iocap_softc *sc;

	sc = device_get_softc(dev);

	VTBLK_LOCK(sc);
	sc->vtblk_iocap_flags |= VTBLK_FLAG_DETACH;
	if (device_is_attached(dev))
		vtblk_iocap_stop(sc);
	VTBLK_UNLOCK(sc);

	vtblk_iocap_drain(sc);

	if (sc->vtblk_iocap_disk != NULL) {
		disk_destroy(sc->vtblk_iocap_disk);
		sc->vtblk_iocap_disk = NULL;
	}

	if (sc->vtblk_iocap_request_tag != NULL) {
		bus_dma_tag_destroy((bus_dma_tag_t)sc->vtblk_iocap_request_tag);
		sc->vtblk_iocap_request_tag = NULL;
	}

	if (sc->vtblk_iocap_queues_tag != NULL) {
		bus_dma_tag_destroy((bus_dma_tag_t)sc->vtblk_iocap_queues_tag);
		sc->vtblk_iocap_queues_tag = NULL;
	}

	if (sc->vtblk_iocap_sglist != NULL) {
		sglist_free(sc->vtblk_iocap_sglist);
		sc->vtblk_iocap_sglist = NULL;
	}

	VTBLK_LOCK_DESTROY(sc);

	return (0);
}

static int
vtblk_iocap_suspend(device_t dev)
{
	struct vtblk_iocap_softc *sc;
	int error;

	sc = device_get_softc(dev);

	VTBLK_LOCK(sc);
	sc->vtblk_iocap_flags |= VTBLK_FLAG_SUSPEND;
	/* XXX BMV: virtio_stop(), etc needed here? */
	error = vtblk_iocap_quiesce(sc);
	if (error)
		sc->vtblk_iocap_flags &= ~VTBLK_FLAG_SUSPEND;
	VTBLK_UNLOCK(sc);

	return (error);
}

static int
vtblk_iocap_resume(device_t dev)
{
	struct vtblk_iocap_softc *sc;

	sc = device_get_softc(dev);

	VTBLK_LOCK(sc);
	/* XXX BMV: virtio_reinit(), etc needed here? */
	sc->vtblk_iocap_flags &= ~VTBLK_FLAG_SUSPEND;
	vtblk_iocap_startio(sc);
	VTBLK_UNLOCK(sc);

	return (0);
}

static int
vtblk_iocap_shutdown(device_t dev)
{

	return (0);
}

static int
vtblk_iocap_attach_completed(device_t dev)
{
	struct vtblk_iocap_softc *sc;

	sc = device_get_softc(dev);

	/*
	 * Create disk after attach as VIRTIO_BLK_T_GET_ID can only be
	 * processed after the device acknowledged
	 * VIRTIO_CONFIG_STATUS_DRIVER_OK.
	 */
	vtblk_iocap_create_disk(sc);
	return (0);
}

static int
vtblk_iocap_config_change(device_t dev)
{
	struct vtblk_iocap_softc *sc;
	struct virtio_blk_config blkcfg;
	uint64_t capacity;

	sc = device_get_softc(dev);

	vtblk_iocap_read_config(sc, &blkcfg);

	/* Capacity is always in 512-byte units. */
	capacity = blkcfg.capacity * VTBLK_BSIZE;

	if (sc->vtblk_iocap_disk->d_mediasize != capacity)
		vtblk_iocap_resize_disk(sc, capacity);

	return (0);
}

static int
vtblk_iocap_open(struct disk *dp)
{
	struct vtblk_iocap_softc *sc;

	if ((sc = dp->d_drv1) == NULL)
		return (ENXIO);

	return (sc->vtblk_iocap_flags & VTBLK_FLAG_DETACH ? ENXIO : 0);
}

static int
vtblk_iocap_close(struct disk *dp)
{
	struct vtblk_iocap_softc *sc;

	if ((sc = dp->d_drv1) == NULL)
		return (ENXIO);

	return (0);
}

static int
vtblk_iocap_ioctl(struct disk *dp, u_long cmd, void *addr, int flag,
    struct thread *td)
{
	struct vtblk_iocap_softc *sc;

	if ((sc = dp->d_drv1) == NULL)
		return (ENXIO);

	return (ENOTTY);
}

static int
vtblk_iocap_dump(void *arg, void *virtual, off_t offset, size_t length)
{
	struct disk *dp;
	struct vtblk_iocap_softc *sc;
	int error;

	dp = arg;
	error = 0;

	if ((sc = dp->d_drv1) == NULL)
		return (ENXIO);

	VTBLK_LOCK(sc);

	vtblk_iocap_dump_quiesce(sc);

	if (length > 0)
		error = vtblk_iocap_dump_write(sc, virtual, offset, length);
	if (error || (virtual == NULL && offset == 0))
		vtblk_iocap_dump_complete(sc);

	VTBLK_UNLOCK(sc);

	return (error);
}

static void
vtblk_iocap_strategy(struct bio *bp)
{
	struct vtblk_iocap_softc *sc;

	if ((sc = bp->bio_disk->d_drv1) == NULL) {
		vtblk_iocap_bio_done(NULL, bp, EINVAL);
		return;
	}

	if ((bp->bio_cmd != BIO_READ) && (bp->bio_cmd != BIO_WRITE) &&
	    (bp->bio_cmd != BIO_FLUSH) && (bp->bio_cmd != BIO_DELETE)) {
		vtblk_iocap_bio_done(sc, bp, EOPNOTSUPP);
		return;
	}

	VTBLK_LOCK(sc);

	if (sc->vtblk_iocap_flags & VTBLK_FLAG_DETACH) {
		VTBLK_UNLOCK(sc);
		vtblk_iocap_bio_done(sc, bp, ENXIO);
		return;
	}

	bioq_insert_tail(&sc->vtblk_iocap_bioq, bp);
	vtblk_iocap_startio(sc);

	VTBLK_UNLOCK(sc);
}

static int
vtblk_iocap_negotiate_features(struct vtblk_iocap_softc *sc)
{
	device_t dev;
	uint64_t features;

	dev = sc->vtblk_iocap_dev;
	features = virtio_bus_is_modern(dev) ? VTBLK_MODERN_FEATURES :
	    VTBLK_LEGACY_FEATURES;

	sc->vtblk_iocap_features = virtio_negotiate_features(dev, features);
	return (virtio_finalize_features(dev));
}

static int
vtblk_iocap_setup_features(struct vtblk_iocap_softc *sc)
{
	device_t dev;
	int error;

	dev = sc->vtblk_iocap_dev;

	error = vtblk_iocap_negotiate_features(sc);
	if (error)
		return (error);

	if (virtio_with_feature(dev, VIRTIO_RING_F_INDIRECT_DESC))
		sc->vtblk_iocap_flags |= VTBLK_FLAG_INDIRECT;
	if (virtio_with_feature(dev, VIRTIO_BLK_F_CONFIG_WCE))
		sc->vtblk_iocap_flags |= VTBLK_FLAG_WCE_CONFIG;

	/* Legacy. */
	if (virtio_with_feature(dev, VIRTIO_BLK_F_BARRIER))
		sc->vtblk_iocap_flags |= VTBLK_FLAG_BARRIER;

	return (0);
}

static int
vtblk_iocap_maximum_segments(struct vtblk_iocap_softc *sc,
    struct virtio_blk_config *blkcfg)
{
	device_t dev;
	int nsegs;

	dev = sc->vtblk_iocap_dev;
	nsegs = VTBLK_MIN_SEGMENTS;

	if (virtio_with_feature(dev, VIRTIO_BLK_F_SEG_MAX)) {
		nsegs += MIN(blkcfg->seg_max, maxphys / PAGE_SIZE + 1);
		if (sc->vtblk_iocap_flags & VTBLK_FLAG_INDIRECT)
			nsegs = MIN(nsegs, VIRTIO_MAX_INDIRECT);
	} else
		nsegs += 1;

	return (nsegs);
}

static int
vtblk_iocap_alloc_virtqueue(struct vtblk_iocap_softc *sc)
{
	device_t dev;
	struct vq_iocap_alloc_info vq_info;

	dev = sc->vtblk_iocap_dev;

	VQ_IOCAP_ALLOC_INFO_INIT(&vq_info, sc->vtblk_iocap_max_nsegs,
	    vtblk_iocap_vq_intr, sc, &sc->vtblk_iocap_vq,
	    "%s request", device_get_nameunit(dev));

	return (virtio_alloc_iocap_virtqueues(dev, sc->vtblk_iocap_queues_tag, 1, &vq_info));
}

static void
vtblk_iocap_resize_disk(struct vtblk_iocap_softc *sc, uint64_t new_capacity)
{
	device_t dev;
	struct disk *dp;
	int error;

	dev = sc->vtblk_iocap_dev;
	dp = sc->vtblk_iocap_disk;

	dp->d_mediasize = new_capacity;
	if (bootverbose) {
		device_printf(dev, "resized to %juMB (%ju %u byte sectors)\n",
		    (uintmax_t) dp->d_mediasize >> 20,
		    (uintmax_t) dp->d_mediasize / dp->d_sectorsize,
		    dp->d_sectorsize);
	}

	error = disk_resize(dp, M_NOWAIT);
	if (error) {
		device_printf(dev,
		    "disk_resize(9) failed, error: %d\n", error);
	}
}

static void
vtblk_iocap_alloc_disk(struct vtblk_iocap_softc *sc, struct virtio_blk_config *blkcfg)
{
	device_t dev;
	struct disk *dp;

	dev = sc->vtblk_iocap_dev;

	sc->vtblk_iocap_disk = dp = disk_alloc();
	dp->d_open = vtblk_iocap_open;
	dp->d_close = vtblk_iocap_close;
	dp->d_ioctl = vtblk_iocap_ioctl;
	dp->d_strategy = vtblk_iocap_strategy;
	dp->d_name = VTBLK_DISK_NAME;
	dp->d_unit = device_get_unit(dev);
	dp->d_drv1 = sc;
	dp->d_flags = DISKFLAG_UNMAPPED_BIO | DISKFLAG_DIRECT_COMPLETION;
	dp->d_hba_vendor = virtio_get_vendor(dev);
	dp->d_hba_device = virtio_get_device(dev);
	dp->d_hba_subvendor = virtio_get_subvendor(dev);
	dp->d_hba_subdevice = virtio_get_subdevice(dev);

	if (virtio_with_feature(dev, VIRTIO_BLK_F_RO))
		dp->d_flags |= DISKFLAG_WRITE_PROTECT;
	else {
		if (virtio_with_feature(dev, VIRTIO_BLK_F_FLUSH))
			dp->d_flags |= DISKFLAG_CANFLUSHCACHE;
		dp->d_dump = vtblk_iocap_dump;
	}

	/* Capacity is always in 512-byte units. */
	dp->d_mediasize = blkcfg->capacity * VTBLK_BSIZE;

	if (virtio_with_feature(dev, VIRTIO_BLK_F_BLK_SIZE))
		dp->d_sectorsize = blkcfg->blk_size;
	else
		dp->d_sectorsize = VTBLK_BSIZE;

	/*
	 * The VirtIO maximum I/O size is given in terms of segments.
	 * However, FreeBSD limits I/O size by logical buffer size, not
	 * by physically contiguous pages. Therefore, we have to assume
	 * no pages are contiguous. This may impose an artificially low
	 * maximum I/O size. But in practice, since QEMU advertises 128
	 * segments, this gives us a maximum IO size of 125 * PAGE_SIZE,
	 * which is typically greater than maxphys. Eventually we should
	 * just advertise maxphys and split buffers that are too big.
	 *
	 * If we're not asking busdma to align data to page boundaries, the
	 * maximum I/O size is reduced by PAGE_SIZE in order to accommodate
	 * unaligned I/Os.
	 */
	dp->d_maxsize = (sc->vtblk_iocap_max_nsegs - VTBLK_MIN_SEGMENTS) *
	    PAGE_SIZE;
	if ((sc->vtblk_iocap_flags & VTBLK_FLAG_BUSDMA_ALIGN) == 0)
		dp->d_maxsize -= PAGE_SIZE;

	if (virtio_with_feature(dev, VIRTIO_BLK_F_GEOMETRY)) {
		dp->d_fwsectors = blkcfg->geometry.sectors;
		dp->d_fwheads = blkcfg->geometry.heads;
	}

	if (virtio_with_feature(dev, VIRTIO_BLK_F_TOPOLOGY) &&
	    blkcfg->topology.physical_block_exp > 0) {
		dp->d_stripesize = dp->d_sectorsize *
		    (1 << blkcfg->topology.physical_block_exp);
		dp->d_stripeoffset = (dp->d_stripesize -
		    blkcfg->topology.alignment_offset * dp->d_sectorsize) %
		    dp->d_stripesize;
	}

	if (virtio_with_feature(dev, VIRTIO_BLK_F_DISCARD)) {
		dp->d_flags |= DISKFLAG_CANDELETE;
		dp->d_delmaxsize = blkcfg->max_discard_sectors * VTBLK_BSIZE;
	}

	if (vtblk_iocap_write_cache_enabled(sc, blkcfg) != 0)
		sc->vtblk_iocap_write_cache = VTBLK_CACHE_WRITEBACK;
	else
		sc->vtblk_iocap_write_cache = VTBLK_CACHE_WRITETHROUGH;
}

static void
vtblk_iocap_create_disk(struct vtblk_iocap_softc *sc)
{
	struct disk *dp;

	dp = sc->vtblk_iocap_disk;

	vtblk_iocap_ident(sc);

	device_printf(sc->vtblk_iocap_dev, "%juMB (%ju %u byte sectors)\n",
	    (uintmax_t) dp->d_mediasize >> 20,
	    (uintmax_t) dp->d_mediasize / dp->d_sectorsize,
	    dp->d_sectorsize);

	disk_create(dp, DISK_VERSION);
}

static int
vtblk_iocap_request_prealloc(struct vtblk_iocap_softc *sc)
{
	struct vtblk_iocap_request *req;
	int i, nreqs;

	nreqs = virtq_iocap_size(sc->vtblk_iocap_vq);

	/*
	 * Preallocate sufficient requests to keep the virtqueue full. Each
	 * request consumes VTBLK_MIN_SEGMENTS or more descriptors so reduce
	 * the number allocated when indirect descriptors are not available.
	 */
	if ((sc->vtblk_iocap_flags & VTBLK_FLAG_INDIRECT) == 0)
		nreqs /= VTBLK_MIN_SEGMENTS;

	for (i = 0; i < nreqs; i++) {
		req = malloc(sizeof(struct vtblk_iocap_request), M_DEVBUF, M_NOWAIT);
		if (req == NULL)
			return (ENOMEM);

		req->vbr_sc = sc;
		if (bus_dmamap_create((bus_dma_tag_t)sc->vtblk_iocap_request_tag, 0, &req->vbr_mapp)) {
			free(req, M_DEVBUF);
			return (ENOMEM);
		}
		if (!bus_dmamap_can_mint_iocap(req->vbr_mapp)) {
			device_printf(sc->vtblk_iocap_dev, "tried to prealloc a dmamap that can't mint IOCaps\n");
			return (EINVAL);
		}

		MPASS(sglist_count(&req->vbr_hdr, sizeof(req->vbr_hdr)) == 1);
		MPASS(sglist_count(&req->vbr_ack, sizeof(req->vbr_ack)) == 1);

		sc->vtblk_iocap_request_count++;
		vtblk_iocap_request_enqueue(sc, req);
	}

	return (0);
}

static void
vtblk_iocap_request_free(struct vtblk_iocap_softc *sc)
{
	struct vtblk_iocap_request *req;

	MPASS(TAILQ_EMPTY(&sc->vtblk_iocap_req_ready));

	while ((req = vtblk_iocap_request_dequeue(sc)) != NULL) {
		sc->vtblk_iocap_request_count--;
		bus_dmamap_destroy((bus_dma_tag_t)sc->vtblk_iocap_request_tag, req->vbr_mapp);
		free(req, M_DEVBUF);
	}

	KASSERT(sc->vtblk_iocap_request_count == 0,
	    ("%s: leaked %d requests", __func__, sc->vtblk_iocap_request_count));
}

static struct vtblk_iocap_request *
vtblk_iocap_request_dequeue(struct vtblk_iocap_softc *sc)
{
	struct vtblk_iocap_request *req;

	req = TAILQ_FIRST(&sc->vtblk_iocap_req_free);
	if (req != NULL) {
		TAILQ_REMOVE(&sc->vtblk_iocap_req_free, req, vbr_link);
		bzero(&req->vbr_hdr, sizeof(struct vtblk_iocap_request) -
		    offsetof(struct vtblk_iocap_request, vbr_hdr));
	}

	return (req);
}

static void
vtblk_iocap_request_enqueue(struct vtblk_iocap_softc *sc, struct vtblk_iocap_request *req)
{

	TAILQ_INSERT_HEAD(&sc->vtblk_iocap_req_free, req, vbr_link);
}

static struct vtblk_iocap_request *
vtblk_iocap_request_next_ready(struct vtblk_iocap_softc *sc)
{
	struct vtblk_iocap_request *req;

	req = TAILQ_FIRST(&sc->vtblk_iocap_req_ready);
	if (req != NULL)
		TAILQ_REMOVE(&sc->vtblk_iocap_req_ready, req, vbr_link);

	return (req);
}

static void
vtblk_iocap_request_requeue_ready(struct vtblk_iocap_softc *sc, struct vtblk_iocap_request *req)
{

	/* NOTE: Currently, there will be at most one request in the queue. */
	TAILQ_INSERT_HEAD(&sc->vtblk_iocap_req_ready, req, vbr_link);
}

static struct vtblk_iocap_request *
vtblk_iocap_request_next(struct vtblk_iocap_softc *sc)
{
	struct vtblk_iocap_request *req;

	req = vtblk_iocap_request_next_ready(sc);
	if (req != NULL)
		return (req);

	return (vtblk_iocap_request_bio(sc));
}

static struct vtblk_iocap_request *
vtblk_iocap_request_bio(struct vtblk_iocap_softc *sc)
{
	struct bio_queue_head *bioq;
	struct vtblk_iocap_request *req;
	struct bio *bp;

	bioq = &sc->vtblk_iocap_bioq;

	if (bioq_first(bioq) == NULL)
		return (NULL);

	req = vtblk_iocap_request_dequeue(sc);
	if (req == NULL)
		return (NULL);

	bp = bioq_takefirst(bioq);
	req->vbr_bp = bp;
	req->vbr_ack = -1;
	req->vbr_hdr.ioprio = vtblk_iocap_gtoh32(sc, 1);

	switch (bp->bio_cmd) {
	case BIO_FLUSH:
		req->vbr_hdr.type = vtblk_iocap_gtoh32(sc, VIRTIO_BLK_T_FLUSH);
		req->vbr_hdr.sector = 0;
		break;
	case BIO_READ:
		req->vbr_hdr.type = vtblk_iocap_gtoh32(sc, VIRTIO_BLK_T_IN);
		req->vbr_hdr.sector = vtblk_iocap_gtoh64(sc, bp->bio_offset / VTBLK_BSIZE);
		break;
	case BIO_WRITE:
		req->vbr_hdr.type = vtblk_iocap_gtoh32(sc, VIRTIO_BLK_T_OUT);
		req->vbr_hdr.sector = vtblk_iocap_gtoh64(sc, bp->bio_offset / VTBLK_BSIZE);
		break;
	case BIO_DELETE:
		req->vbr_hdr.type = vtblk_iocap_gtoh32(sc, VIRTIO_BLK_T_DISCARD);
		req->vbr_hdr.sector = vtblk_iocap_gtoh64(sc, bp->bio_offset / VTBLK_BSIZE);
		break;
	default:
		panic("%s: bio with unhandled cmd: %d", __func__, bp->bio_cmd);
	}

	if (bp->bio_flags & BIO_ORDERED)
		req->vbr_hdr.type |= vtblk_iocap_gtoh32(sc, VIRTIO_BLK_T_BARRIER);

	return (req);
}

static int
vtblk_iocap_request_execute(struct vtblk_iocap_request *req, int flags)
{
	struct vtblk_iocap_softc *sc = req->vbr_sc;
	struct bio *bp = req->vbr_bp;
	int error = 0;

	/*
	 * Call via bus_dmamap_load_bio or directly depending on whether we
	 * have a buffer we need to map.  If we don't have a busdma map,
	 * try to perform the I/O directly and hope that it works (this will
	 * happen when dumping).
	 */
	if ((req->vbr_mapp != NULL) &&
	    (bp->bio_cmd == BIO_READ || bp->bio_cmd == BIO_WRITE)) {
		error = bus_dmamap_load_bio((bus_dma_tag_t)sc->vtblk_iocap_request_tag, req->vbr_mapp,
		    req->vbr_bp, vtblk_iocap_request_execute_cb, req, flags);
		if (error == EINPROGRESS) {
			req->vbr_busdma_wait = 1;
			sc->vtblk_iocap_flags |= VTBLK_FLAG_BUSDMA_WAIT;
		}
	} else {
		vtblk_iocap_request_execute_cb(req, NULL, 0, 0);
	}

	return (error ? error : req->vbr_error);
}

static void
vtblk_iocap_request_execute_cb(void * callback_arg, bus_dma_segment_t * segs,
    int nseg, int error)
{
	struct vtblk_iocap_request *req;
	struct vtblk_iocap_softc *sc;
	struct virtq_iocap *vq;
	struct sglist *sg;
	struct bio *bp;
	int ordered, readable, writable, i;

	req = (struct vtblk_iocap_request *)callback_arg;
	sc = req->vbr_sc;
	vq = sc->vtblk_iocap_vq;
	sg = sc->vtblk_iocap_sglist;
	bp = req->vbr_bp;
	ordered = 0;
	writable = 0;

	/*
	 * If we paused request queueing while we waited for busdma to call us
	 * asynchronously, unpause it now; this request made it through so we
	 * don't need to worry about others getting ahead of us.  (Note that we
	 * hold the device mutex so nothing will happen until after we return
	 * anyway.)
	 */
	if (req->vbr_busdma_wait)
		sc->vtblk_iocap_flags &= ~VTBLK_FLAG_BUSDMA_WAIT;

	/* Fail on errors from busdma. */
	if (error)
		goto out1;

	/*
	 * Some hosts (such as bhyve) do not implement the barrier feature,
	 * so we emulate it in the driver by allowing the barrier request
	 * to be the only one in flight.
	 */
	if ((sc->vtblk_iocap_flags & VTBLK_FLAG_BARRIER) == 0) {
		if (sc->vtblk_iocap_req_ordered != NULL) {
			error = EBUSY;
			goto out;
		}
		if (bp->bio_flags & BIO_ORDERED) {
			if (!virtq_iocap_empty(vq)) {
				error = EBUSY;
				goto out;
			}
			ordered = 1;
			req->vbr_hdr.type &= vtblk_iocap_gtoh32(sc,
				~VIRTIO_BLK_T_BARRIER);
		}
	}

	sglist_reset(sg);
	sglist_append(sg, &req->vbr_hdr, sizeof(struct virtio_blk_outhdr));

	if (bp->bio_cmd == BIO_READ || bp->bio_cmd == BIO_WRITE) {
		/*
		 * We cast bus_addr_t to vm_paddr_t here; since we skip the
		 * iommu mapping (see vtblk_iocap_attach) this should be safe.
		 */
		for (i = 0; i < nseg; i++) {
			error = sglist_append_phys(sg,
			    (vm_paddr_t)segs[i].ds_addr, segs[i].ds_len);
			// device_printf(sc->vtblk_iocap_dev, "sglist_append_phys 0x%lx 0x%lx\n", segs[i].ds_addr, segs[i].ds_len);
			if (error || sg->sg_nseg == sg->sg_maxseg) {
				panic("%s: bio %p data buffer too big %d",
				    __func__, bp, error);
			}
		}

		/* Special handling for dump, which bypasses busdma. */
		if (req->vbr_mapp == NULL) {
			error = sglist_append_bio(sg, bp);
			if (error || sg->sg_nseg == sg->sg_maxseg) {
				panic("%s: bio %p data buffer too big %d",
				    __func__, bp, error);
			}
		}

		/* BIO_READ means the host writes into our buffer. */
		if (bp->bio_cmd == BIO_READ)
			writable = sg->sg_nseg - 1;
	} else if (bp->bio_cmd == BIO_DELETE) {
		struct virtio_blk_discard_write_zeroes *discard;

		discard = malloc(sizeof(*discard), M_DEVBUF, M_NOWAIT | M_ZERO);
		if (discard == NULL) {
			error = ENOMEM;
			goto out;
		}

		bp->bio_driver1 = discard;
		discard->sector = vtblk_iocap_gtoh64(sc, bp->bio_offset / VTBLK_BSIZE);
		discard->num_sectors = vtblk_iocap_gtoh32(sc, bp->bio_bcount / VTBLK_BSIZE);
		error = sglist_append(sg, discard, sizeof(*discard));
		if (error || sg->sg_nseg == sg->sg_maxseg) {
			panic("%s: bio %p data buffer too big %d",
			    __func__, bp, error);
		}
	}

	writable++;
	sglist_append(sg, &req->vbr_ack, sizeof(uint8_t));
	readable = sg->sg_nseg - writable;

	if (req->vbr_mapp != NULL) {
		switch (bp->bio_cmd) {
		case BIO_READ:
			bus_dmamap_sync((bus_dma_tag_t)sc->vtblk_iocap_request_tag, req->vbr_mapp,
			    BUS_DMASYNC_PREREAD);
			break;
		case BIO_WRITE:
			bus_dmamap_sync((bus_dma_tag_t)sc->vtblk_iocap_request_tag, req->vbr_mapp,
			    BUS_DMASYNC_PREWRITE);
			break;
		}
	}

	error = virtq_iocap_enqueue(vq, req->vbr_iocap_mapp, req, sg, readable, writable);
	if (error == 0 && ordered)
		sc->vtblk_iocap_req_ordered = req;

	/*
	 * If we were called asynchronously, we need to notify the queue that
	 * we've added a new request, since the notification from startio was
	 * performed already.
	 */
	if (error == 0 && req->vbr_busdma_wait)
		virtq_iocap_notify(vq);

out:
	if (error && (req->vbr_mapp != NULL)) {
		// If error != 0, the mapping hasn't actualy been mapped into the device's view
		// or allocated an IOCap key yet.
		// Therefore we can call bus_dmamap_unload to *synchronously*
		// unmap without waiting for IOCap key epochs.
		bus_dmamap_unload((bus_dma_tag_t)sc->vtblk_iocap_request_tag, req->vbr_mapp);
	}

out1:
	if (error && req->vbr_requeue_on_error)
		vtblk_iocap_request_requeue_ready(sc, req);
	req->vbr_error = error;
}

static int
vtblk_iocap_request_error(struct vtblk_iocap_request *req)
{
	int error;

	switch (req->vbr_ack) {
	case VIRTIO_BLK_S_OK:
		error = 0;
		break;
	case VIRTIO_BLK_S_UNSUPP:
		error = ENOTSUP;
		break;
	default:
		error = EIO;
		break;
	}

	return (error);
}

typedef void (*vtblk_unload_cb)(struct vtblk_iocap_request *req, void* cb_arg2);

// bus_dmamap_sync the memory under a request and unmap it from the device's view.
// Immediately after this function returns, the memory it points to is usable, but it may still be accessible from the device.
// Once the callback is called, the memory has been removed from the device view
// and is safe to free and reuse for other purposes.
static void
vtblk_iocap_queue_complete_one(struct vtblk_iocap_softc *sc, struct vtblk_iocap_request *req, vtblk_unload_cb cb, void* cb_arg2)
{
	struct bio *bp;

	if (sc->vtblk_iocap_req_ordered != NULL) {
		MPASS(sc->vtblk_iocap_req_ordered == req);
		sc->vtblk_iocap_req_ordered = NULL;
	}

	bp = req->vbr_bp;
	bp->bio_error = vtblk_iocap_request_error(req);
	if (req->vbr_mapp != NULL) {
		switch (bp->bio_cmd) {
		case BIO_READ:
			bus_dmamap_sync((bus_dma_tag_t)sc->vtblk_iocap_request_tag, req->vbr_mapp,
			    BUS_DMASYNC_POSTREAD);
			iocap_keymngr_bus_dmamap_unload2(
				sc->vtblk_iocap_request_tag,
				req->vbr_iocap_mapp,
				(iocap_keymngr_bus_dmamap_unload2_cb)cb, req, cb_arg2
				);
			break;
		case BIO_WRITE:
			bus_dmamap_sync((bus_dma_tag_t)sc->vtblk_iocap_request_tag, req->vbr_mapp,
			    BUS_DMASYNC_POSTWRITE);
			iocap_keymngr_bus_dmamap_unload2(
				sc->vtblk_iocap_request_tag,
				req->vbr_iocap_mapp,
				(iocap_keymngr_bus_dmamap_unload2_cb)cb, req, cb_arg2
				);
			break;
		}
	}
}

static void
vtblk_iocap_queue_request_unloaded(struct vtblk_iocap_request *req, void* cb_arg2)
{
	struct bio *bp;
	struct bio_queue *queue;

	bp = req->vbr_bp;
	queue = (struct bio_queue*) cb_arg2;

	TAILQ_INSERT_TAIL(queue, bp, bio_queue);
	vtblk_iocap_request_enqueue(req->vbr_sc, req);
}

static void
vtblk_iocap_queue_completed(struct vtblk_iocap_softc *sc, struct bio_queue *queue)
{
	struct vtblk_iocap_request *req;

	while ((req = virtq_iocap_dequeue(sc->vtblk_iocap_vq, NULL)) != NULL) {
		vtblk_iocap_queue_complete_one(sc, req, vtblk_iocap_queue_request_unloaded, queue);
	}
}

static void
vtblk_iocap_done_completed(struct vtblk_iocap_softc *sc, struct bio_queue *queue)
{
	struct bio *bp, *tmp;

	TAILQ_FOREACH_SAFE(bp, queue, bio_queue, tmp) {
		if (bp->bio_error != 0)
			disk_err(bp, "hard error", -1, 1);
		vtblk_iocap_bio_done(sc, bp, bp->bio_error);
	}
}

static void
vtblk_iocap_drain_vq(struct vtblk_iocap_softc *sc)
{
	struct virtq_iocap *vq;
	struct vtblk_iocap_request *req;
	int last;

	vq = sc->vtblk_iocap_vq;
	last = 0;

	while ((req = virtq_iocap_drain(vq, &last)) != NULL) {
		vtblk_iocap_bio_done(sc, req->vbr_bp, ENXIO);
		vtblk_iocap_request_enqueue(sc, req);
	}

	sc->vtblk_iocap_req_ordered = NULL;
	KASSERT(virtq_iocap_empty(vq), ("virtq_iocap not empty"));
}

static void
vtblk_iocap_drain(struct vtblk_iocap_softc *sc)
{
	struct bio_queue_head *bioq;
	struct vtblk_iocap_request *req;
	struct bio *bp;

	bioq = &sc->vtblk_iocap_bioq;

	if (sc->vtblk_iocap_vq != NULL) {
		struct bio_queue queue;

		TAILQ_INIT(&queue);
		vtblk_iocap_queue_completed(sc, &queue);
		vtblk_iocap_done_completed(sc, &queue);

		vtblk_iocap_drain_vq(sc);
	}

	while ((req = vtblk_iocap_request_next_ready(sc)) != NULL) {
		vtblk_iocap_bio_done(sc, req->vbr_bp, ENXIO);
		vtblk_iocap_request_enqueue(sc, req);
	}

	while (bioq_first(bioq) != NULL) {
		bp = bioq_takefirst(bioq);
		vtblk_iocap_bio_done(sc, bp, ENXIO);
	}

	vtblk_iocap_request_free(sc);
}

static void
vtblk_iocap_startio(struct vtblk_iocap_softc *sc)
{
	struct virtq_iocap *vq;
	struct vtblk_iocap_request *req;
	int enq;

	VTBLK_LOCK_ASSERT(sc);
	vq = sc->vtblk_iocap_vq;
	enq = 0;

	if (sc->vtblk_iocap_flags & (VTBLK_FLAG_SUSPEND | VTBLK_FLAG_BUSDMA_WAIT))
		return;

	while (!virtq_iocap_full(vq)) {
		req = vtblk_iocap_request_next(sc);
		if (req == NULL)
			break;

		req->vbr_requeue_on_error = 1;
		if (vtblk_iocap_request_execute(req, BUS_DMA_WAITOK))
			break;

		enq++;
	}

	if (enq > 0)
		virtq_iocap_notify(vq);
}

static void
vtblk_iocap_bio_done(struct vtblk_iocap_softc *sc, struct bio *bp, int error)
{

	/* Because of GEOM direct dispatch, we cannot hold any locks. */
	if (sc != NULL)
		VTBLK_LOCK_ASSERT_NOTOWNED(sc);

	if (error) {
		bp->bio_resid = bp->bio_bcount;
		bp->bio_error = error;
		bp->bio_flags |= BIO_ERROR;
	} else {
		kmsan_mark_bio(bp, KMSAN_STATE_INITED);
	}

	if (bp->bio_driver1 != NULL) {
		free(bp->bio_driver1, M_DEVBUF);
		bp->bio_driver1 = NULL;
	}

	biodone(bp);
}

#define VTBLK_GET_CONFIG(_dev, _feature, _field, _cfg)			\
	if (virtio_with_feature(_dev, _feature)) {			\
		virtio_read_device_config(_dev,				\
		    offsetof(struct virtio_blk_config, _field),		\
		    &(_cfg)->_field, sizeof((_cfg)->_field));		\
	}

static void
vtblk_iocap_read_config(struct vtblk_iocap_softc *sc, struct virtio_blk_config *blkcfg)
{
	device_t dev;

	dev = sc->vtblk_iocap_dev;

	bzero(blkcfg, sizeof(struct virtio_blk_config));

	/* The capacity is always available. */
	virtio_read_device_config(dev, offsetof(struct virtio_blk_config,
	    capacity), &blkcfg->capacity, sizeof(blkcfg->capacity));

	/* Read the configuration if the feature was negotiated. */
	VTBLK_GET_CONFIG(dev, VIRTIO_BLK_F_SIZE_MAX, size_max, blkcfg);
	VTBLK_GET_CONFIG(dev, VIRTIO_BLK_F_SEG_MAX, seg_max, blkcfg);
	VTBLK_GET_CONFIG(dev, VIRTIO_BLK_F_GEOMETRY,
	    geometry.cylinders, blkcfg);
	VTBLK_GET_CONFIG(dev, VIRTIO_BLK_F_GEOMETRY,
	    geometry.heads, blkcfg);
	VTBLK_GET_CONFIG(dev, VIRTIO_BLK_F_GEOMETRY,
	    geometry.sectors, blkcfg);
	VTBLK_GET_CONFIG(dev, VIRTIO_BLK_F_BLK_SIZE, blk_size, blkcfg);
	VTBLK_GET_CONFIG(dev, VIRTIO_BLK_F_TOPOLOGY,
	    topology.physical_block_exp, blkcfg);
	VTBLK_GET_CONFIG(dev, VIRTIO_BLK_F_TOPOLOGY,
	    topology.alignment_offset, blkcfg);
	VTBLK_GET_CONFIG(dev, VIRTIO_BLK_F_TOPOLOGY,
	    topology.min_io_size, blkcfg);
	VTBLK_GET_CONFIG(dev, VIRTIO_BLK_F_TOPOLOGY,
	    topology.opt_io_size, blkcfg);
	VTBLK_GET_CONFIG(dev, VIRTIO_BLK_F_CONFIG_WCE, wce, blkcfg);
	VTBLK_GET_CONFIG(dev, VIRTIO_BLK_F_DISCARD, max_discard_sectors,
	    blkcfg);
	VTBLK_GET_CONFIG(dev, VIRTIO_BLK_F_DISCARD, max_discard_seg, blkcfg);
	VTBLK_GET_CONFIG(dev, VIRTIO_BLK_F_DISCARD, discard_sector_alignment,
	    blkcfg);
}

#undef VTBLK_GET_CONFIG

static void
vtblk_iocap_ident(struct vtblk_iocap_softc *sc)
{
	struct bio buf;
	struct disk *dp;
	struct vtblk_iocap_request *req;
	int len, error;

	dp = sc->vtblk_iocap_disk;
	len = MIN(VIRTIO_BLK_ID_BYTES, DISK_IDENT_SIZE);

	if (vtblk_iocap_tunable_int(sc, "no_ident", vtblk_iocap_no_ident) != 0)
		return;

	req = vtblk_iocap_request_dequeue(sc);
	if (req == NULL)
		return;

	req->vbr_ack = -1;
	req->vbr_hdr.type = vtblk_iocap_gtoh32(sc, VIRTIO_BLK_T_GET_ID);
	req->vbr_hdr.ioprio = vtblk_iocap_gtoh32(sc, 1);
	req->vbr_hdr.sector = 0;

	req->vbr_bp = &buf;
	g_reset_bio(&buf);

	buf.bio_cmd = BIO_READ;
	buf.bio_data = dp->d_ident;
	buf.bio_bcount = len;

	// TODO Make this asynchronous, we really don't want to hand out
	// full access to this struct
	VTBLK_LOCK(sc);
	error = vtblk_iocap_poll_request(sc, req);
	VTBLK_UNLOCK(sc);

	if (error) {
		device_printf(sc->vtblk_iocap_dev,
		    "error getting device identifier: %d\n", error);
	}
}

// THIS FUNCTION IS FAKE AND FOR THINGS THAT REALLY DONT CARE ABOUT MEMORY SECURITY
static int
vtblk_iocap_poll_request(struct vtblk_iocap_softc *sc, struct vtblk_iocap_request *req)
{
	struct vtblk_iocap_request *req1 __diagused;
	struct virtq_iocap *vq;
	struct bio *bp;
	int error;

	vq = sc->vtblk_iocap_vq;

	if (!virtq_iocap_empty(vq))
		return (EBUSY);

	error = vtblk_iocap_request_execute(req, BUS_DMA_NOWAIT);
	if (error)
		return (error);

	virtq_iocap_notify(vq);
	req1 = virtq_iocap_poll(vq, NULL);
	KASSERT(req == req1,
	    ("%s: polling completed %p not %p", __func__, req1, req));

	// NOTE THIS SETS CALLBACK TO NULL BECAUSE IT DOESNT CARE ABOUT MEMORY SECURITY
	vtblk_iocap_queue_complete_one(sc, req, NULL, NULL);
	bp = req->vbr_bp;
	error = bp->bio_error;
	if (error && bootverbose) {
		device_printf(sc->vtblk_iocap_dev,
		    "%s: IO error: %d\n", __func__, error);
	}
	if (req != &sc->vtblk_iocap_dump_request)
		vtblk_iocap_request_enqueue(sc, req);

	return (error);
}

static int
vtblk_iocap_quiesce(struct vtblk_iocap_softc *sc)
{
	int error;

	VTBLK_LOCK_ASSERT(sc);
	error = 0;

	while (!virtq_iocap_empty(sc->vtblk_iocap_vq)) {
		if (mtx_sleep(&sc->vtblk_iocap_vq, VTBLK_MTX(sc), PRIBIO, "vtblk_iocapq",
		    VTBLK_QUIESCE_TIMEOUT) == EWOULDBLOCK) {
			error = EBUSY;
			break;
		}
	}

	return (error);
}

static void
vtblk_iocap_vq_intr(void *xsc)
{
	struct vtblk_iocap_softc *sc;
	struct virtq_iocap *vq;
	struct bio_queue queue;

	sc = xsc;
	vq = sc->vtblk_iocap_vq;
	TAILQ_INIT(&queue);

	VTBLK_LOCK(sc);

again:
	if (sc->vtblk_iocap_flags & VTBLK_FLAG_DETACH)
		goto out;

	vtblk_iocap_queue_completed(sc, &queue);
	vtblk_iocap_startio(sc);

	if (virtq_iocap_enable_intr(vq) != 0) {
		virtq_iocap_disable_intr(vq);
		goto again;
	}

	if (sc->vtblk_iocap_flags & VTBLK_FLAG_SUSPEND)
		wakeup(&sc->vtblk_iocap_vq);

out:
	VTBLK_UNLOCK(sc);
	vtblk_iocap_done_completed(sc, &queue);
}

static void
vtblk_iocap_stop(struct vtblk_iocap_softc *sc)
{

	virtq_iocap_disable_intr(sc->vtblk_iocap_vq);
	virtio_stop(sc->vtblk_iocap_dev);
}

static void
vtblk_iocap_dump_quiesce(struct vtblk_iocap_softc *sc)
{

	/*
	 * Spin here until all the requests in-flight at the time of the
	 * dump are completed and queued. The queued requests will be
	 * biodone'd once the dump is finished.
	 */
	while (!virtq_iocap_empty(sc->vtblk_iocap_vq))
		vtblk_iocap_queue_completed(sc, &sc->vtblk_iocap_dump_queue);
}

static int
vtblk_iocap_dump_write(struct vtblk_iocap_softc *sc, void *virtual, off_t offset,
    size_t length)
{
	struct bio buf;
	struct vtblk_iocap_request *req;

	req = &sc->vtblk_iocap_dump_request;
	req->vbr_sc = sc;
	req->vbr_ack = -1;
	req->vbr_hdr.type = vtblk_iocap_gtoh32(sc, VIRTIO_BLK_T_OUT);
	req->vbr_hdr.ioprio = vtblk_iocap_gtoh32(sc, 1);
	req->vbr_hdr.sector = vtblk_iocap_gtoh64(sc, offset / VTBLK_BSIZE);

	req->vbr_bp = &buf;
	g_reset_bio(&buf);

	buf.bio_cmd = BIO_WRITE;
	buf.bio_data = virtual;
	buf.bio_bcount = length;

	return (vtblk_iocap_poll_request(sc, req));
}

static int
vtblk_iocap_dump_flush(struct vtblk_iocap_softc *sc)
{
	struct bio buf;
	struct vtblk_iocap_request *req;

	req = &sc->vtblk_iocap_dump_request;
	req->vbr_sc = sc;
	req->vbr_ack = -1;
	req->vbr_hdr.type = vtblk_iocap_gtoh32(sc, VIRTIO_BLK_T_FLUSH);
	req->vbr_hdr.ioprio = vtblk_iocap_gtoh32(sc, 1);
	req->vbr_hdr.sector = 0;

	req->vbr_bp = &buf;
	g_reset_bio(&buf);

	buf.bio_cmd = BIO_FLUSH;

	return (vtblk_iocap_poll_request(sc, req));
}

static void
vtblk_iocap_dump_complete(struct vtblk_iocap_softc *sc)
{

	vtblk_iocap_dump_flush(sc);

	VTBLK_UNLOCK(sc);
	vtblk_iocap_done_completed(sc, &sc->vtblk_iocap_dump_queue);
	VTBLK_LOCK(sc);
}

static void
vtblk_iocap_set_write_cache(struct vtblk_iocap_softc *sc, int wc)
{

	/* Set either writeback (1) or writethrough (0) mode. */
	virtio_write_dev_config_1(sc->vtblk_iocap_dev,
	    offsetof(struct virtio_blk_config, wce), wc);
}

static int
vtblk_iocap_write_cache_enabled(struct vtblk_iocap_softc *sc,
    struct virtio_blk_config *blkcfg)
{
	int wc;

	if (sc->vtblk_iocap_flags & VTBLK_FLAG_WCE_CONFIG) {
		wc = vtblk_iocap_tunable_int(sc, "writecache_mode",
		    vtblk_iocap_writecache_mode);
		if (wc >= 0 && wc < VTBLK_CACHE_MAX)
			vtblk_iocap_set_write_cache(sc, wc);
		else
			wc = blkcfg->wce;
	} else
		wc = virtio_with_feature(sc->vtblk_iocap_dev, VIRTIO_BLK_F_FLUSH);

	return (wc);
}

static int
vtblk_iocap_write_cache_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct vtblk_iocap_softc *sc;
	int wc, error;

	sc = oidp->oid_arg1;
	wc = sc->vtblk_iocap_write_cache;

	error = sysctl_handle_int(oidp, &wc, 0, req);
	if (error || req->newptr == NULL)
		return (error);
	if ((sc->vtblk_iocap_flags & VTBLK_FLAG_WCE_CONFIG) == 0)
		return (EPERM);
	if (wc < 0 || wc >= VTBLK_CACHE_MAX)
		return (EINVAL);

	VTBLK_LOCK(sc);
	sc->vtblk_iocap_write_cache = wc;
	vtblk_iocap_set_write_cache(sc, sc->vtblk_iocap_write_cache);
	VTBLK_UNLOCK(sc);

	return (0);
}

static void
vtblk_iocap_setup_sysctl(struct vtblk_iocap_softc *sc)
{
	device_t dev;
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *tree;
	struct sysctl_oid_list *child;

	dev = sc->vtblk_iocap_dev;
	ctx = device_get_sysctl_ctx(dev);
	tree = device_get_sysctl_tree(dev);
	child = SYSCTL_CHILDREN(tree);

	SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "writecache_mode",
	    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 0,
	    vtblk_iocap_write_cache_sysctl, "I",
	    "Write cache mode (writethrough (0) or writeback (1))");
}

static int
vtblk_iocap_tunable_int(struct vtblk_iocap_softc *sc, const char *knob, int def)
{
	char path[64];

	snprintf(path, sizeof(path),
	    "hw.vtblk_iocap.%d.%s", device_get_unit(sc->vtblk_iocap_dev), knob);
	TUNABLE_INT_FETCH(path, &def);

	return (def);
}
