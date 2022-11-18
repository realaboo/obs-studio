#pragma once

#include "c99defs.h"
#include "bitstream.h"

/*
 *   Exp golomb decoding
 */

#ifdef __cplusplus
extern "C" {
#endif

/* Read an unsigned Exp-Golomb code from a bitstream */
EXPORT uint32_t read_exp_golomb(struct bitstream_reader *r);
EXPORT int32_t read_sexp_golomb(struct bitstream_reader *r);

#ifdef __cplusplus
}
#endif
#pragma once
