// This file is included at the top of libccap.c.
// It allows the header dependencies of libccap to be swapped out
// if certain headers are unavailable (e.g. in kernel development standard <string.h> may not be available)
// or if the libccap.h header itself is in a subfolder or should be accessed through a wrapper.
// For example, if <stdbool.h> or <stdint.h> are not available a wrapper header should be created like so:
// ```
// #define LIBCCAP_NO_STANDARD_HEADERS
// #include "equivalent/to/stdbool.h"
// #include "equivalent/to/stdint.h"
// #include "path/to/real/libccap.h"
// ```
// and both this file and any files trying to use libccap should #include that wrapper file instead of libccap.h directly.
// You may also define the extern functions here, stubs are provided below.

#include <dev/iocap/iocap.h>

// If you would like to debug the generation or decoding of iocaps, you can uncomment these defines and includes.
// #include <stdio.h>
// #define libccap_dbg_trace(...) printf(__VA_ARGS__)

// libccap depends on having memcmp available.
// by default that is provided by <string.h>, but this can be swapped out if not present.
// If the function is named differently, you can redirect it by defining LIBCCAP_MEMCMP.
#include <sys/systm.h>
#define LIBCCAP_MEMCMP(lhs, rhs, count) memcmp(lhs, rhs, count)

// By default, libccap defines functions internal to the libccap translation unit as 'static'
// and does not apply any extra modifiers to external functions.
// Both can be overridden here.
// #define LIBCCAP_INTERNAL_FUNC_PREFIX static
// #define LIBCCAP_EXTERNAL_FUNC_PREFIX

// libccap uses an ilog2() function, which may be provided by an external header but by default is defined internally.
// The default internal definition can be overridden here.
#define LIBCCAP_ILOG2(x) ilog2(x)

// STUBS
#include <crypto/rijndael/rijndael.h>

void ccap_aes_encrypt_128_func(const CCapU128* secret, const CCapU128* data, CCapU128* result) {
    rijndael_ctx ctx;
    rijndael_set_key(&ctx, *secret, 128);
    rijndael_encrypt(&ctx, *data, *result);
}

uint64_t ccap_panic_write_utf8(const uint8_t *utf8, uint64_t utf_len) {
    // TODO kernel logging?
    return utf_len;
}

void ccap_panic_complete(void) {
    // TODO kernel panic?
}

// libccap uses the _Noreturn (C11 onwards) or [[noreturn]] (C23 onwards) specifiers to mark a function as non-returning, based on the __STDC_VERSION__.
// This is imporant to avoid spurious uninitialized variable warnings - in certain impossible cases libccap jumps to a non-returning panic() function,
// and if C believes the panic funcion *could* return then it would complain about certain state that cannot be initialized correctly.
// It can be overridden here.
// #define LIBCCAP_NORETURN_SPECIFIER _Noreturn

