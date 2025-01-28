// Minorly based on virtio_ring.h
#ifndef VIRTIO_RING_IOCAP_H
#define	VIRTIO_RING_IOCAP_H

#include <dev/virtio/virtio_ring.h>
#include <dev/iocap/iocap.h>

struct vring_iocap {
	unsigned int num;

	struct iocap *desc;
	struct vring_avail *avail;
	struct vring_used *used;
};

/* The standard layout for the ring is a continuous chunk of memory which
 * looks like this.  We assume num is a power of 2.
 *
 * struct vring {
 *      // The actual descriptors (32 bytes each)
 *      struct iocap desc[num];
 *
 *      // A ring of available descriptor heads with free-running index.
 *      __u16 avail_flags;
 *      __u16 avail_idx;
 *      __u16 available[num];
 *      __u16 used_event_idx;
 *
 *      // Padding to the next align boundary.
 *      char pad[];
 *
 *      // A ring of used descriptor heads with free-running index.
 *      __u16 used_flags;
 *      __u16 used_idx;
 *      struct vring_used_elem used[num];
 *      __u16 avail_event_idx;
 * };
 *
 * NOTE: for VirtIO PCI, align is 4096.
 */

static inline int
vring_iocap_size(unsigned int num, unsigned long align)
{
	int size;

	size = num * sizeof(struct iocap);
	size += sizeof(struct vring_avail) + (num * sizeof(uint16_t)) +
	    sizeof(uint16_t);
	size = (size + align - 1) & ~(align - 1);
	size += sizeof(struct vring_used) +
	    (num * sizeof(struct vring_used_elem)) + sizeof(uint16_t);
	return (size);
}

static inline void
vring_iocap_init(struct vring_iocap *vr, unsigned int num, uint8_t *p,
    unsigned long align)
{
        vr->num = num;
        vr->desc = (struct iocap *) p;
        vr->avail = (struct vring_avail *) (p +
	    num * sizeof(struct iocap));
        vr->used = (void *)roundup2(&vr->avail->ring[num], align);
}

#endif