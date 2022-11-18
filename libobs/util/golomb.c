#include "golomb.h"

uint32_t read_exp_golomb(struct bitstream_reader *r)
{
	uint32_t leading_zeros = 0;
	while (!bitstream_reader_read_bit(r))
		leading_zeros++;

	uint32_t code = 1;
	for (uint32_t i = 0; i < leading_zeros; i++)
		code = (code << 1) | bitstream_reader_read_bit(r);

	return code - 1;
}

int32_t read_sexp_golomb(struct bitstream_reader* r)
{
	uint32_t buf = read_exp_golomb(r);
	int32_t sign = (buf & 1) - 1;
	return ((buf >> 1) ^ sign) + 1;
}
