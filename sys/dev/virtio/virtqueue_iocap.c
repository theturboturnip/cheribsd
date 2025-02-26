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

/*
 * Implements the virtqueue interface as basically described
 * in the original VirtIO paper.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/sdt.h>
#include <sys/sglist.h>
#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/cpu.h>
#include <machine/bus.h>
#include <machine/atomic.h>
#include <machine/resource.h>
#include <sys/bus.h>
#include <sys/rman.h>

#include <dev/iocap/iocap_keymngr.h>
#include <dev/virtio/virtio.h>
#include <dev/virtio/virtqueue_iocap.h>
#include <dev/virtio/virtio_ring_iocap.h>

#include "virtio_bus_if.h"

struct virtq_iocap {
	device_t		 vq_dev;
	uint16_t		 vq_queue_index;
	uint16_t		 vq_nentries;
	uint32_t		 vq_flags;
#define	VIRTQ_IOCAP_FLAG_MODERN	 0x0001
#define	VIRTQ_IOCAP_FLAG_INDIRECT	 0x0002
#define	VIRTQ_IOCAP_FLAG_EVENT_IDX 0x0004

	int			 vq_max_indirect_size;
	bus_size_t		 vq_notify_offset;
	virtqueue_intr_t	*vq_intrhand;
	void			*vq_intrhand_arg;

	struct vring_iocap		 vq_ring;
	uint16_t		 vq_free_cnt;
	uint16_t		 vq_queued_cnt;
	/*
	 * Head of the free chain in the descriptor table. If
	 * there are no free descriptors, this will be set to
	 * VQ_IOCAP_RING_DESC_CHAIN_END.
	 */
	uint16_t		 vq_desc_head_idx;
	/*
	 * Last consumed descriptor in the used table,
	 * trails vq_ring.used->idx.
	 */
	uint16_t		 vq_used_cons_idx;

	void			*vq_iocap_ring_mem;
	int			 vq_indirect_mem_size;
	int			 vq_alignment;
	int			 vq_iocap_ring_size;
	char			 vq_name[VIRTQ_IOCAP_MAX_NAME_SZ];

	struct vq_desc_extra {
		void		  *cookie;
		struct iocap *indirect;
		vm_paddr_t	   indirect_paddr;
		uint16_t	   ndescs;
	} vq_descx[0];
};

/*
 * The maximum virtqueue IOCap *next* value is 2^13 - 1. Use that value as the end of
 * descriptor chain terminator since it will never be a valid index
 * in the descriptor table. This is used to verify we are correctly
 * handling vq_free_cnt.
 */
#define VQ_IOCAP_RING_DESC_CHAIN_END 8191

#define VQASSERT(_vq, _exp, _msg, ...)				\
    KASSERT((_exp),("%s: %s - "_msg, __func__, (_vq)->vq_name,	\
	##__VA_ARGS__))

#define VQASSERT_CCAPRESULT(_vq, _res, _msg)		     \
	VQASSERT((_vq), ((_res) == CCapResult_Success), "ccap error %s while %s", \
	ccap_result_str((_res)), (_msg))

#define VQ_IOCAP_RING_ASSERT_VALID_IDX(_vq, _idx)			\
    VQASSERT((_vq), (_idx) < (_vq)->vq_nentries,		\
	"invalid ring index: %d, max: %d", (_idx),		\
	(_vq)->vq_nentries)

#define VQ_IOCAP_RING_ASSERT_CHAIN_TERM(_vq)				\
    VQASSERT((_vq), (_vq)->vq_desc_head_idx ==			\
	VQ_IOCAP_RING_DESC_CHAIN_END,	"full ring terminated "		\
	"incorrectly: head idx: %d", (_vq)->vq_desc_head_idx)

// Equivalent of VQASSERT_CCAPRESULT that doesn't turn off in debug builds
#define VQREQUIRE_CCAPRESULT(_vq, _res, _msg, ...) do {				\
	if (__predict_false((_res) != CCapResult_Success))			\
		panic("%s: %s - %s - "_msg, __func__, (_vq)->vq_name,		\
			ccap_result_str(_res), ##__VA_ARGS__);			\
} while (0)


static int	virtq_iocap_init_indirect(struct virtq_iocap *vq, int);
static void	virtq_iocap_free_indirect(struct virtq_iocap *vq);
static void	virtq_iocap_init_indirect_list(struct virtq_iocap *,
		    struct iocap *);

static void	vq_iocap_ring_init(struct virtq_iocap *);
static void	vq_iocap_ring_update_avail(struct virtq_iocap *, uint16_t);
static uint16_t	vq_iocap_ring_enqueue_segments(struct virtq_iocap *,
	bus_iocap_dmamap_t, struct iocap *, uint16_t, struct sglist *, int, int);
static bool	vq_iocap_ring_use_indirect(struct virtq_iocap *, int);
static void	vq_iocap_ring_enqueue_indirect(struct virtq_iocap *,
	bus_iocap_dmamap_t, void *, struct sglist *, int, int);
static int	vq_iocap_ring_enable_interrupt(struct virtq_iocap *, uint16_t);
static int	vq_iocap_ring_must_notify_host(struct virtq_iocap *);
static void	vq_iocap_ring_notify_host(struct virtq_iocap *);
static void	vq_iocap_ring_free_chain(struct virtq_iocap *, uint16_t);

SDT_PROVIDER_DEFINE(virtqueue);
SDT_PROBE_DEFINE6(virtqueue, , enqueue_segments, entry, "struct virtq_iocap *",
    "struct iocap *", "uint16_t", "struct sglist *", "int", "int");
SDT_PROBE_DEFINE1(virtqueue, , enqueue_segments, return, "uint16_t");

#define vq_modern(_vq) 		(((_vq)->vq_flags & VIRTQ_IOCAP_FLAG_MODERN) != 0)
#define vq_htog16(_vq, _val) 	virtio_htog16(vq_modern(_vq), _val)
#define vq_htog32(_vq, _val) 	virtio_htog32(vq_modern(_vq), _val)
#define vq_htog64(_vq, _val) 	virtio_htog64(vq_modern(_vq), _val)
#define vq_gtoh16(_vq, _val) 	virtio_gtoh16(vq_modern(_vq), _val)
#define vq_gtoh32(_vq, _val) 	virtio_gtoh32(vq_modern(_vq), _val)
#define vq_gtoh64(_vq, _val) 	virtio_gtoh64(vq_modern(_vq), _val)

int
virtq_iocap_alloc(device_t dev, uint16_t queue, uint16_t size,
    bus_size_t notify_offset, int align, vm_paddr_t highaddr,
    struct vq_iocap_alloc_info *info, struct virtq_iocap **vqp)
{
	struct virtq_iocap *vq;
	int error;

	*vqp = NULL;
	error = 0;

	if (size == 0) {
		device_printf(dev,
		    "virtqueue %d (%s) does not exist (size is zero)\n",
		    queue, info->vqai_name);
		return (ENODEV);
	} else if (!powerof2(size)) {
		device_printf(dev,
		    "virtqueue %d (%s) size is not a power of 2: %d\n",
		    queue, info->vqai_name, size);
		return (ENXIO);
	} else if (size > VQ_IOCAP_RING_DESC_CHAIN_END) {
		device_printf(dev,
		    "virtqueue %d (%s) size is too large - 'next' fields would exceed 2^13-1: %d\n",
		    queue, info->vqai_name, size);
		return (EDOM);
	} else if (info->vqai_maxindirsz > VIRTIO_MAX_INDIRECT) {
		device_printf(dev, "virtqueue %d (%s) requested too many "
		    "indirect descriptors: %d, max %d\n",
		    queue, info->vqai_name, info->vqai_maxindirsz,
		    VIRTIO_MAX_INDIRECT);
		return (EINVAL);
	}

	vq = malloc(sizeof(struct virtq_iocap) +
	    size * sizeof(struct vq_desc_extra), M_DEVBUF, M_NOWAIT | M_ZERO);
	if (vq == NULL) {
		device_printf(dev, "cannot allocate virtqueue\n");
		return (ENOMEM);
	}

	vq->vq_dev = dev;
	strlcpy(vq->vq_name, info->vqai_name, sizeof(vq->vq_name));
	vq->vq_queue_index = queue;
	vq->vq_notify_offset = notify_offset;
	vq->vq_alignment = align;
	vq->vq_nentries = size;
	vq->vq_free_cnt = size;
	vq->vq_intrhand = info->vqai_intr;
	vq->vq_intrhand_arg = info->vqai_intr_arg;

	if (VIRTIO_BUS_WITH_FEATURE(dev, VIRTIO_F_VERSION_1) != 0)
		vq->vq_flags |= VIRTQ_IOCAP_FLAG_MODERN;
	if (VIRTIO_BUS_WITH_FEATURE(dev, VIRTIO_RING_F_EVENT_IDX) != 0)
		vq->vq_flags |= VIRTQ_IOCAP_FLAG_EVENT_IDX;

	if (info->vqai_maxindirsz > 1) {
		error = virtq_iocap_init_indirect(vq, info->vqai_maxindirsz);
		if (error)
			goto fail;
	}

	vq->vq_iocap_ring_size = round_page(vring_iocap_size(size, align));
	vq->vq_iocap_ring_mem = contigmalloc(vq->vq_iocap_ring_size, M_DEVBUF,
	    M_NOWAIT | M_ZERO, 0, highaddr, PAGE_SIZE, 0);
	if (vq->vq_iocap_ring_mem == NULL) {
		device_printf(dev,
		    "cannot allocate memory for virtqueue ring\n");
		error = ENOMEM;
		goto fail;
	}

	vq_iocap_ring_init(vq);
	virtq_iocap_disable_intr(vq);

	*vqp = vq;

fail:
	if (error)
		virtq_iocap_free(vq);

	return (error);
}

static int
virtq_iocap_init_indirect(struct virtq_iocap *vq, int indirect_size)
{
	device_t dev;
	struct vq_desc_extra *dxp;
	int i, size;

	dev = vq->vq_dev;

	if (VIRTIO_BUS_WITH_FEATURE(dev, VIRTIO_RING_F_INDIRECT_DESC) == 0) {
		/*
		 * Indirect descriptors requested by the driver but not
		 * negotiated. Return zero to keep the initialization
		 * going: we'll run fine without.
		 */
		if (bootverbose)
			device_printf(dev, "virtqueue %d (%s) requested "
			    "indirect descriptors but not negotiated\n",
			    vq->vq_queue_index, vq->vq_name);
		return (0);
	}

	if (indirect_size >= VQ_IOCAP_RING_DESC_CHAIN_END) {
		device_printf(dev,
		    "virtqueue %d (%s) indirect_size is too large - 'next' fields would exceed 2^13-1: %d\n",
		    vq->vq_queue_index, vq->vq_name, indirect_size);
		return (EDOM);
	}

	size = indirect_size * sizeof(struct iocap);
	vq->vq_max_indirect_size = indirect_size;
	vq->vq_indirect_mem_size = size;
	vq->vq_flags |= VIRTQ_IOCAP_FLAG_INDIRECT;

	for (i = 0; i < vq->vq_nentries; i++) {
		dxp = &vq->vq_descx[i];

		dxp->indirect = malloc(size, M_DEVBUF, M_NOWAIT);
		if (dxp->indirect == NULL) {
			device_printf(dev, "cannot allocate indirect list\n");
			return (ENOMEM);
		}

		dxp->indirect_paddr = vtophys(dxp->indirect);
		virtq_iocap_init_indirect_list(vq, dxp->indirect);
	}

	return (0);
}

static void
virtq_iocap_free_indirect(struct virtq_iocap *vq)
{
	struct vq_desc_extra *dxp;
	int i;

	for (i = 0; i < vq->vq_nentries; i++) {
		dxp = &vq->vq_descx[i];

		if (dxp->indirect == NULL)
			break;

		free(dxp->indirect, M_DEVBUF);
		dxp->indirect = NULL;
		dxp->indirect_paddr = 0;
	}

	vq->vq_flags &= ~VIRTQ_IOCAP_FLAG_INDIRECT;
	vq->vq_indirect_mem_size = 0;
}

static void
virtq_iocap_init_indirect_list(struct virtq_iocap *vq,
    struct iocap *indirect)
{
	int i;
	CCapResult res __diagused; // This is used in assertions that should never fire

	bzero(indirect, vq->vq_indirect_mem_size);

	for (i = 0; i < vq->vq_max_indirect_size - 1; i++) {
		res = clear_iocap_virtio_except_next(&indirect[i], i + 1);
		VQASSERT_CCAPRESULT(vq, res, "virtq_iocap_init_indirect_list");
	}
	res = clear_iocap_virtio_except_next(&indirect[i], VQ_IOCAP_RING_DESC_CHAIN_END);
	VQASSERT_CCAPRESULT(vq, res, "virtq_iocap_init_indirect_list final");
}

int
virtq_iocap_reinit(struct virtq_iocap *vq, uint16_t size)
{
	struct vq_desc_extra *dxp;
	int i;

	if (vq->vq_nentries != size) {
		device_printf(vq->vq_dev,
		    "%s: '%s' changed size; old=%hu, new=%hu\n",
		    __func__, vq->vq_name, vq->vq_nentries, size);
		return (EINVAL);
	}

	/* Warn if the virtqueue was not properly cleaned up. */
	if (vq->vq_free_cnt != vq->vq_nentries) {
		device_printf(vq->vq_dev,
		    "%s: warning '%s' virtqueue not empty, "
		    "leaking %d entries\n", __func__, vq->vq_name,
		    vq->vq_nentries - vq->vq_free_cnt);
	}

	vq->vq_desc_head_idx = 0;
	vq->vq_used_cons_idx = 0;
	vq->vq_queued_cnt = 0;
	vq->vq_free_cnt = vq->vq_nentries;

	/* To be safe, reset all our allocated memory. */
	bzero(vq->vq_iocap_ring_mem, vq->vq_iocap_ring_size);
	for (i = 0; i < vq->vq_nentries; i++) {
		dxp = &vq->vq_descx[i];
		dxp->cookie = NULL;
		dxp->ndescs = 0;
		if (vq->vq_flags & VIRTQ_IOCAP_FLAG_INDIRECT)
			virtq_iocap_init_indirect_list(vq, dxp->indirect);
	}

	vq_iocap_ring_init(vq);
	virtq_iocap_disable_intr(vq);

	return (0);
}

void
virtq_iocap_free(struct virtq_iocap *vq)
{

	if (vq->vq_free_cnt != vq->vq_nentries) {
		device_printf(vq->vq_dev, "%s: freeing non-empty virtqueue, "
		    "leaking %d entries\n", vq->vq_name,
		    vq->vq_nentries - vq->vq_free_cnt);
	}

	if (vq->vq_flags & VIRTQ_IOCAP_FLAG_INDIRECT)
		virtq_iocap_free_indirect(vq);

	if (vq->vq_iocap_ring_mem != NULL) {
		free(vq->vq_iocap_ring_mem, M_DEVBUF);
		vq->vq_iocap_ring_size = 0;
		vq->vq_iocap_ring_mem = NULL;
	}

	free(vq, M_DEVBUF);
}

vm_paddr_t
virtq_iocap_paddr(struct virtq_iocap *vq)
{

	return (vtophys(vq->vq_iocap_ring_mem));
}

vm_paddr_t
virtq_iocap_desc_paddr(struct virtq_iocap *vq)
{

	return (vtophys(vq->vq_ring.desc));
}

vm_paddr_t
virtq_iocap_avail_paddr(struct virtq_iocap *vq)
{

	return (vtophys(vq->vq_ring.avail));
}

vm_paddr_t
virtq_iocap_used_paddr(struct virtq_iocap *vq)
{

	return (vtophys(vq->vq_ring.used));
}

size_t
virtq_iocap_size_bytes(struct virtq_iocap *vq)
{
	return (size_t)vq->vq_iocap_ring_size;
}

uint16_t
virtq_iocap_index(struct virtq_iocap *vq)
{

	return (vq->vq_queue_index);
}

int
virtq_iocap_size(struct virtq_iocap *vq)
{

	return (vq->vq_nentries);
}

int
virtq_iocap_nfree(struct virtq_iocap *vq)
{

	return (vq->vq_free_cnt);
}

bool
virtq_iocap_empty(struct virtq_iocap *vq)
{

	return (vq->vq_nentries == vq->vq_free_cnt);
}

bool
virtq_iocap_full(struct virtq_iocap *vq)
{

	return (vq->vq_free_cnt == 0);
}

void
virtq_iocap_notify(struct virtq_iocap *vq)
{

	/* Ensure updated avail->idx is visible to host. */
	mb();

	if (vq_iocap_ring_must_notify_host(vq))
		vq_iocap_ring_notify_host(vq);
	vq->vq_queued_cnt = 0;
}

int
virtq_iocap_nused(struct virtq_iocap *vq)
{
	uint16_t used_idx, nused;

	used_idx = vq_htog16(vq, vq->vq_ring.used->idx);

	nused = (uint16_t)(used_idx - vq->vq_used_cons_idx);
	VQASSERT(vq, nused <= vq->vq_nentries, "used more than available");

	return (nused);
}

int
virtq_iocap_intr_filter(struct virtq_iocap *vq)
{

	if (vq->vq_used_cons_idx == vq_htog16(vq, vq->vq_ring.used->idx))
		return (0);

	virtq_iocap_disable_intr(vq);

	return (1);
}

void
virtq_iocap_intr(struct virtq_iocap *vq)
{

	vq->vq_intrhand(vq->vq_intrhand_arg);
}

int
virtq_iocap_enable_intr(struct virtq_iocap *vq)
{

	return (vq_iocap_ring_enable_interrupt(vq, 0));
}

int
virtq_iocap_postpone_intr(struct virtq_iocap *vq, vq_postpone_t hint)
{
	uint16_t ndesc, avail_idx;

	avail_idx = vq_htog16(vq, vq->vq_ring.avail->idx);
	ndesc = (uint16_t)(avail_idx - vq->vq_used_cons_idx);

	switch (hint) {
	case VQ_POSTPONE_SHORT:
		ndesc = ndesc / 4;
		break;
	case VQ_POSTPONE_LONG:
		ndesc = (ndesc * 3) / 4;
		break;
	case VQ_POSTPONE_EMPTIED:
		break;
	}

	return (vq_iocap_ring_enable_interrupt(vq, ndesc));
}

/*
 * Note this is only considered a hint to the host.
 */
void
virtq_iocap_disable_intr(struct virtq_iocap *vq)
{

	if (vq->vq_flags & VIRTQ_IOCAP_FLAG_EVENT_IDX) {
		vring_used_event(&vq->vq_ring) = vq_gtoh16(vq,
		    vq->vq_used_cons_idx - vq->vq_nentries - 1);
		return;
	}

	vq->vq_ring.avail->flags |= vq_gtoh16(vq, VRING_AVAIL_F_NO_INTERRUPT);
}

int
virtq_iocap_enqueue(struct virtq_iocap *vq, bus_iocap_dmamap_t mapp, void *cookie, struct sglist *sg,
    int readable, int writable)
{
	struct vq_desc_extra *dxp;
	int needed;
	uint16_t head_idx, idx;

	needed = readable + writable;

	VQASSERT(vq, cookie != NULL, "enqueuing with no cookie");
	VQASSERT(vq, needed == sg->sg_nseg,
	    "segment count mismatch, %d, %d", needed, sg->sg_nseg);
	VQASSERT(vq,
	    needed <= vq->vq_nentries || needed <= vq->vq_max_indirect_size,
	    "too many segments to enqueue: %d, %d/%d", needed,
	    vq->vq_nentries, vq->vq_max_indirect_size);

	if (needed < 1)
		return (EINVAL);
	if (vq->vq_free_cnt == 0)
		return (ENOSPC);

	if (vq_iocap_ring_use_indirect(vq, needed)) {
		vq_iocap_ring_enqueue_indirect(vq, mapp, cookie, sg, readable, writable);
		return (0);
	} else if (vq->vq_free_cnt < needed)
		return (EMSGSIZE);

	head_idx = vq->vq_desc_head_idx;
	VQ_IOCAP_RING_ASSERT_VALID_IDX(vq, head_idx);
	dxp = &vq->vq_descx[head_idx];

	VQASSERT(vq, dxp->cookie == NULL,
	    "cookie already exists for index %d", head_idx);
	dxp->cookie = cookie;
	dxp->ndescs = needed;

	idx = vq_iocap_ring_enqueue_segments(vq, mapp, vq->vq_ring.desc, head_idx,
	    sg, readable, writable);

	vq->vq_desc_head_idx = idx;
	vq->vq_free_cnt -= needed;
	if (vq->vq_free_cnt == 0)
		VQ_IOCAP_RING_ASSERT_CHAIN_TERM(vq);
	else
		VQ_IOCAP_RING_ASSERT_VALID_IDX(vq, idx);

	vq_iocap_ring_update_avail(vq, head_idx);

	return (0);
}

void *
virtq_iocap_dequeue(struct virtq_iocap *vq, uint32_t *len)
{
	struct vring_used_elem *uep;
	void *cookie;
	uint16_t used_idx, desc_idx;

	if (vq->vq_used_cons_idx == vq_htog16(vq, vq->vq_ring.used->idx))
		return (NULL);

	used_idx = vq->vq_used_cons_idx++ & (vq->vq_nentries - 1);
	uep = &vq->vq_ring.used->ring[used_idx];

	rmb();
	desc_idx = (uint16_t) vq_htog32(vq, uep->id);
	if (len != NULL)
		*len = vq_htog32(vq, uep->len);

	vq_iocap_ring_free_chain(vq, desc_idx);

	cookie = vq->vq_descx[desc_idx].cookie;
	VQASSERT(vq, cookie != NULL, "no cookie for index %d", desc_idx);
	vq->vq_descx[desc_idx].cookie = NULL;

	return (cookie);
}

void *
virtq_iocap_poll(struct virtq_iocap *vq, uint32_t *len)
{
	void *cookie;

	VIRTIO_BUS_POLL(vq->vq_dev);
	while ((cookie = virtq_iocap_dequeue(vq, len)) == NULL) {
		cpu_spinwait();
		VIRTIO_BUS_POLL(vq->vq_dev);
	}

	return (cookie);
}

void *
virtq_iocap_drain(struct virtq_iocap *vq, int *last)
{
	void *cookie;
	int idx;

	cookie = NULL;
	idx = *last;

	while (idx < vq->vq_nentries && cookie == NULL) {
		if ((cookie = vq->vq_descx[idx].cookie) != NULL) {
			vq->vq_descx[idx].cookie = NULL;
			/* Free chain to keep free count consistent. */
			vq_iocap_ring_free_chain(vq, idx);
		}
		idx++;
	}

	*last = idx;

	return (cookie);
}

void
virtq_iocap_dump(struct virtq_iocap *vq)
{

	if (vq == NULL)
		return;

	printf("VQ: %s - size=%d; free=%d; used=%d; queued=%d; "
	    "desc_head_idx=%d; avail.idx=%d; used_cons_idx=%d; "
	    "used.idx=%d; used_event_idx=%d; avail.flags=0x%x; used.flags=0x%x\n",
	    vq->vq_name, vq->vq_nentries, vq->vq_free_cnt, virtq_iocap_nused(vq),
	    vq->vq_queued_cnt, vq->vq_desc_head_idx,
	    vq_htog16(vq, vq->vq_ring.avail->idx), vq->vq_used_cons_idx,
	    vq_htog16(vq, vq->vq_ring.used->idx),
	    vq_htog16(vq, vring_used_event(&vq->vq_ring)),
	    vq_htog16(vq, vq->vq_ring.avail->flags),
	    vq_htog16(vq, vq->vq_ring.used->flags));
}

static void
vq_iocap_ring_init(struct virtq_iocap *vq)
{
	struct vring_iocap *vr;
	uint8_t *ring_mem;
	int i, size;
	CCapResult res __diagused; // This is used in assertions that should never fire

	ring_mem = vq->vq_iocap_ring_mem;
	size = vq->vq_nentries;
	vr = &vq->vq_ring;

	vring_iocap_init(vr, size, ring_mem, vq->vq_alignment);

	for (i = 0; i < size - 1; i++) {
		res = clear_iocap_virtio_except_next(&vr->desc[i], i + 1);
		VQASSERT_CCAPRESULT(vq, res, "vq_iocap_ring_init");
	}
	res = clear_iocap_virtio_except_next(&vr->desc[i], VQ_IOCAP_RING_DESC_CHAIN_END);
	VQASSERT_CCAPRESULT(vq, res, "vq_iocap_ring_init final");
}

static void
vq_iocap_ring_update_avail(struct virtq_iocap *vq, uint16_t desc_idx)
{
	uint16_t avail_idx, avail_ring_idx;

	/*
	 * Place the head of the descriptor chain into the next slot and make
	 * it usable to the host. The chain is made available now rather than
	 * deferring to virtq_iocap_notify() in the hopes that if the host is
	 * currently running on another CPU, we can keep it processing the new
	 * descriptor.
	 */
	avail_idx = vq_htog16(vq, vq->vq_ring.avail->idx);
	avail_ring_idx = avail_idx & (vq->vq_nentries - 1);
	vq->vq_ring.avail->ring[avail_ring_idx] = vq_gtoh16(vq, desc_idx);

	wmb();
	vq->vq_ring.avail->idx = vq_gtoh16(vq, avail_idx + 1);

	/* Keep pending count until virtq_iocap_notify(). */
	vq->vq_queued_cnt++;
}

static uint16_t
vq_iocap_ring_enqueue_segments(struct virtq_iocap *vq, bus_iocap_dmamap_t mapp, struct iocap *desc,
    uint16_t head_idx, struct sglist *sg, int readable, int writable)
{
	struct sglist_seg *seg;
	struct iocap* iocap;
	int i, needed;
	uint16_t idx;

	struct bus_dma_segment dma_seg;
	uint16_t next;
	uint16_t flags;

	CCapResult mint_res;

	SDT_PROBE6(virtqueue, , enqueue_segments, entry, vq, desc, head_idx,
	    sg, readable, writable);

	needed = readable + writable;

	for (i = 0, idx = head_idx, seg = sg->sg_segs;
	     i < needed;
	     i++, idx = next, seg++) {
		VQASSERT(vq, idx != VQ_IOCAP_RING_DESC_CHAIN_END,
		    "premature end of free desc chain");

		iocap = &desc[idx];

		dma_seg.ds_addr = seg->ss_paddr;
		dma_seg.ds_len = seg->ss_len;
		next = iocap_virtio_get_encoded_next(iocap);
		flags = 0;

		if (i < needed - 1)
			flags |= VRING_DESC_F_NEXT;
		if (i >= readable)
			flags |= VRING_DESC_F_WRITE;

		mint_res = bus_dmamap_mint_virtio_iocap(
			mapp,
			&dma_seg,
			flags,
			next,
			iocap
		);
		VQREQUIRE_CCAPRESULT(vq, mint_res, "couldn't mint cap of [%lx + %lx)\n",
			dma_seg.ds_addr, dma_seg.ds_len);
	}

	SDT_PROBE1(virtqueue, , enqueue_segments, return, idx);
	return (idx);
}

static bool
vq_iocap_ring_use_indirect(struct virtq_iocap *vq, int needed)
{

	if ((vq->vq_flags & VIRTQ_IOCAP_FLAG_INDIRECT) == 0)
		return (false);

	if (vq->vq_max_indirect_size < needed)
		return (false);

	if (needed < 2)
		return (false);

	return (true);
}

static void
vq_iocap_ring_enqueue_indirect(struct virtq_iocap *vq, bus_iocap_dmamap_t mapp,
	void *cookie, struct sglist *sg, int readable, int writable)
{
	struct iocap *iocap;
	struct vq_desc_extra *dxp;
	int needed;
	uint16_t head_idx;
	uint16_t iocap_next;
	CCapResult mint_res;

	needed = readable + writable;
	VQASSERT(vq, needed <= vq->vq_max_indirect_size,
	    "enqueuing too many indirect descriptors");

	head_idx = vq->vq_desc_head_idx;
	VQ_IOCAP_RING_ASSERT_VALID_IDX(vq, head_idx);
	iocap = &vq->vq_ring.desc[head_idx];
	dxp = &vq->vq_descx[head_idx];

	VQASSERT(vq, dxp->cookie == NULL,
	    "cookie already exists for index %d", head_idx);
	dxp->cookie = cookie;
	dxp->ndescs = 1;

	struct bus_dma_segment dma_seg;
	dma_seg.ds_addr = dxp->indirect_paddr;
	dma_seg.ds_len = needed * sizeof(struct iocap);
	iocap_next = iocap_virtio_get_encoded_next(iocap);
	mint_res = bus_dmamap_mint_virtio_iocap(
		mapp,
		&dma_seg,
		VRING_DESC_F_INDIRECT,
		// Maintain the next pointer even though we don't use it
		// TODO is this necessary?
		iocap_next,
		iocap
	);
	VQREQUIRE_CCAPRESULT(vq, mint_res, "couldn't mint indirect cap of [%lx + %lx)\n",
		dma_seg.ds_addr, dma_seg.ds_len);

	vq_iocap_ring_enqueue_segments(vq, mapp, dxp->indirect, 0,
	    sg, readable, writable);

	vq->vq_desc_head_idx = iocap_next;
	vq->vq_free_cnt--;
	if (vq->vq_free_cnt == 0)
		VQ_IOCAP_RING_ASSERT_CHAIN_TERM(vq);
	else
		VQ_IOCAP_RING_ASSERT_VALID_IDX(vq, vq->vq_desc_head_idx);

	vq_iocap_ring_update_avail(vq, head_idx);
}

static int
vq_iocap_ring_enable_interrupt(struct virtq_iocap *vq, uint16_t ndesc)
{

	/*
	 * Enable interrupts, making sure we get the latest index of
	 * what's already been consumed.
	 */
	if (vq->vq_flags & VIRTQ_IOCAP_FLAG_EVENT_IDX) {
		vring_used_event(&vq->vq_ring) =
		    vq_gtoh16(vq, vq->vq_used_cons_idx + ndesc);
	} else {
		vq->vq_ring.avail->flags &=
		    vq_gtoh16(vq, ~VRING_AVAIL_F_NO_INTERRUPT);
	}

	mb();

	/*
	 * Enough items may have already been consumed to meet our threshold
	 * since we last checked. Let our caller know so it processes the new
	 * entries.
	 */
	if (virtq_iocap_nused(vq) > ndesc)
		return (1);

	return (0);
}

static int
vq_iocap_ring_must_notify_host(struct virtq_iocap *vq)
{
	uint16_t new_idx, prev_idx, event_idx, flags;

	if (vq->vq_flags & VIRTQ_IOCAP_FLAG_EVENT_IDX) {
		new_idx = vq_htog16(vq, vq->vq_ring.avail->idx);
		prev_idx = new_idx - vq->vq_queued_cnt;
		event_idx = vq_htog16(vq, vring_avail_event(&vq->vq_ring));

		return (vring_need_event(event_idx, new_idx, prev_idx) != 0);
	}

	flags = vq->vq_ring.used->flags;
	return ((flags & vq_gtoh16(vq, VRING_USED_F_NO_NOTIFY)) == 0);
}

static void
vq_iocap_ring_notify_host(struct virtq_iocap *vq)
{

	VIRTIO_BUS_NOTIFY_VQ(vq->vq_dev, vq->vq_queue_index,
	    vq->vq_notify_offset);
}

static void
vq_iocap_ring_free_chain(struct virtq_iocap *vq, uint16_t desc_idx)
{
	struct iocap *iocap;
	struct vq_desc_extra *dxp;
	CCapResult res __diagused; // This is used in assertions that should never fire

	VQ_IOCAP_RING_ASSERT_VALID_IDX(vq, desc_idx);
	iocap = &vq->vq_ring.desc[desc_idx];
	dxp = &vq->vq_descx[desc_idx];

	if (vq->vq_free_cnt == 0)
		VQ_IOCAP_RING_ASSERT_CHAIN_TERM(vq);

	vq->vq_free_cnt += dxp->ndescs;
	dxp->ndescs--;

	uint16_t flags = iocap_virtio_get_indirect_next_flags(iocap);

	if ((flags & VRING_DESC_F_INDIRECT) == 0) {
		while (flags & VRING_DESC_F_NEXT) {
			uint16_t next_idx = iocap_virtio_get_encoded_next(iocap);
			VQ_IOCAP_RING_ASSERT_VALID_IDX(vq, next_idx);
			iocap = &vq->vq_ring.desc[next_idx];
			flags = iocap_virtio_get_indirect_next_flags(iocap);
			dxp->ndescs--;
		}
	}

	VQASSERT(vq, dxp->ndescs == 0,
	    "failed to free entire desc chain, remaining: %d", dxp->ndescs);

	/*
	 * We must append the existing free chain, if any, to the end of
	 * newly freed chain. If the virtqueue was completely used, then
	 * head would be VQ_IOCAP_RING_DESC_CHAIN_END (ASSERTed above).
	 */
	res = clear_iocap_virtio_except_next(iocap, vq->vq_desc_head_idx);
	VQASSERT_CCAPRESULT(vq, res, "vq_iocap_ring_free_chain final");
	vq->vq_desc_head_idx = desc_idx;
}
