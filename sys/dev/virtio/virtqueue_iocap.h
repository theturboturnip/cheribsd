/*-
 * Copyright (c) 2025 Samuel Stark <sws35@cam.ac.uk>
 *
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

#ifndef _VIRTIO_VIRTQUEUE_IOCAP_H
#define _VIRTIO_VIRTQUEUE_IOCAP_H

#include <dev/iocap/iocap_keymngr.h>
#include <dev/virtio/virtqueue.h>

struct virtq_iocap;
struct sglist;

#define VIRTQ_IOCAP_MAX_NAME_SZ	32

/* One for each virtqueue the device wishes to allocate. */
struct vq_iocap_alloc_info {
	char		   vqai_name[VIRTQ_IOCAP_MAX_NAME_SZ];
	int		   vqai_maxindirsz;
	// Function called in an ithread context, which is allowed
	// to take mutexes but should avoid lock contention where possible.
	// See BUS_SETUP_INTR(9)
	virtqueue_intr_t  *vqai_intr;
	void		  *vqai_intr_arg;
	struct virtq_iocap **vqai_vq;
};

#define VQ_IOCAP_ALLOC_INFO_INIT(_i,_nsegs,_intr,_arg,_vqp,_str,...) do {	\
	snprintf((_i)->vqai_name, VIRTQ_IOCAP_MAX_NAME_SZ, _str,		\
	    ##__VA_ARGS__);						\
	(_i)->vqai_maxindirsz = (_nsegs);				\
	(_i)->vqai_intr = (_intr);					\
	(_i)->vqai_intr_arg = (_arg);					\
	(_i)->vqai_vq = (_vqp);						\
} while (0)

int	 virtq_iocap_alloc(device_t dev, uint16_t queue, uint16_t size,
	     bus_size_t notify_offset, int align, vm_paddr_t highaddr,
	     struct vq_iocap_alloc_info *info, struct virtq_iocap **vqp);
void	*virtq_iocap_drain(struct virtq_iocap *vq, int *last);
void	 virtq_iocap_free(struct virtq_iocap *vq);
int	 virtq_iocap_reinit(struct virtq_iocap *vq, uint16_t size);

int	 virtq_iocap_intr_filter(struct virtq_iocap *vq);
void	 virtq_iocap_intr(struct virtq_iocap *vq);
int	 virtq_iocap_enable_intr(struct virtq_iocap *vq);
int	 virtq_iocap_postpone_intr(struct virtq_iocap *vq, vq_postpone_t hint);
void	 virtq_iocap_disable_intr(struct virtq_iocap *vq);

/* Get physical address of the virtqueue ring. */
vm_paddr_t virtq_iocap_paddr(struct virtq_iocap *vq);
vm_paddr_t virtq_iocap_desc_paddr(struct virtq_iocap *vq);
vm_paddr_t virtq_iocap_avail_paddr(struct virtq_iocap *vq);
vm_paddr_t virtq_iocap_used_paddr(struct virtq_iocap *vq);
size_t     virtq_iocap_size_bytes(struct virtq_iocap *vq);

uint16_t virtq_iocap_index(struct virtq_iocap *vq);
bool	 virtq_iocap_full(struct virtq_iocap *vq);
bool	 virtq_iocap_empty(struct virtq_iocap *vq);
int	 virtq_iocap_size(struct virtq_iocap *vq);
int	 virtq_iocap_nfree(struct virtq_iocap *vq);
int	 virtq_iocap_nused(struct virtq_iocap *vq);
void	 virtq_iocap_notify(struct virtq_iocap *vq);
void	 virtq_iocap_dump(struct virtq_iocap *vq);

// Does not mint IOCaps unless the return value is 0.
int	 virtq_iocap_enqueue(struct virtq_iocap *vq, bus_iocap_dmamap_t mapp,
	void *cookie, struct sglist *sg, int readable, int writable);
void	*virtq_iocap_dequeue(struct virtq_iocap *vq, uint32_t *len);
void	*virtq_iocap_poll(struct virtq_iocap *vq, uint32_t *len);

#endif /* _VIRTIO_VIRTQUEUE_IOCAP_H */
