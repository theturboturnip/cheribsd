#ifndef IOCAP_H
#define IOCAP_H

#define LIBCCAP_NO_STANDARD_HEADERS
#include <sys/types.h>
#include "libccap/libccap.h"

struct iocap {
    CCap2024_11 cap __aligned(4);
};

inline uint16_t iocap_virtio_get_encoded_next(struct iocap* iocap)
{
	return ccap2024_11_read_virtio_next(&iocap->cap);
}

// Return only the (INDIRECT | NEXT) flags. Guaranteed to succeed.
// Looking up the WRITE flag is not guaranteed to succeed because
// the permissions on the iocap may be invalid.
inline uint16_t iocap_virtio_get_indirect_next_flags(struct iocap* iocap)
{
	return ccap2024_11_read_virtio_flags_indirect_next(&iocap->cap);
}

// Set the given IOCap data (NOT signature) to a cleared state (all 0s) except the field that encodes
// the virtio `next` pointer. This is useful because FreeBSD uses the `next`
// field to encode information in unused descriptors - namely, the index of the next
// unused descriptor.
__attribute__((warn_unused_result))
inline CCapResult clear_iocap_virtio_except_next(struct iocap* iocap, uint16_t next)
{
	return ccap2024_11_clear_and_write_virtio_next(&iocap->cap, next);
}

#endif