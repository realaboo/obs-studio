#include "bitstream.h"

#include <stdlib.h>
#include <string.h>

void bitstream_reader_init(struct bitstream_reader *r, const uint8_t *data,
			   size_t len)
{
	memset(r, 0, sizeof(struct bitstream_reader));
	r->buf = data;
	r->subPos = 0x80;
	r->len = len;
}

uint8_t bitstream_reader_read_bit(struct bitstream_reader *r)
{
	if (r->pos >= r->len)
		return 0;

	uint8_t bit = (*(r->buf + r->pos) & r->subPos) == r->subPos ? 1 : 0;

	r->subPos >>= 0x1;
	if (r->subPos == 0) {
		r->subPos = 0x80;
		r->pos++;
	}

	return bit;
}

uint8_t bitstream_reader_read_bits(struct bitstream_reader *r, int bits)
{
	uint8_t res = 0;

	for (int i = 1; i <= bits; i++) {
		res <<= 1;
		res |= bitstream_reader_read_bit(r);
	}

	return res;
}

uint8_t bitstream_reader_r8(struct bitstream_reader *r)
{
	return bitstream_reader_read_bits(r, 8);
}

uint16_t bitstream_reader_r16(struct bitstream_reader *r)
{
	uint8_t b = bitstream_reader_read_bits(r, 8);
	return ((uint16_t)b << 8) | bitstream_reader_read_bits(r, 8);
}

uint32_t bitstream_reader_r32(struct bitstream_reader *r, int bits)
{
	uint32_t res = 0;

	for (int i = 1; i <= bits; i++) {
		res <<= 1;
		res |= bitstream_reader_read_bit(r);
	}

	return res;
}

uint64_t bitstream_reader_r64(struct bitstream_reader* r, int bits)
{
	uint64_t res = 0;

	for (int i = 1; i <= bits; i++) {
		res <<= 1;
		res |= bitstream_reader_read_bit(r);
	}

	return res;
}

void bitstream_reader_skip(struct bitstream_reader* r, int bits)
{
	r->pos += bits / 8;
	if (r->pos >= r->len) {
		r->pos = r->len;
		return;
	}

	int remaining = bits % 8;
	if (remaining == 0) {
		return;
	}
	if (r->subPos <= (1 << (remaining - 1))) {
		r->pos++;
		r->subPos <<= (8 - remaining);
	} else {
		r->subPos >>= remaining;
	}
}

size_t _log2(uint8_t num)
{
	size_t res = 0;
	while (num >>= 1)
		res++;
	return res;
}

size_t bitstream_reader_get_bits_left(struct bitstream_reader *r)
{
	return r->pos >= r->len ? 0
				: (r->len - r->pos) * 8 + _log2(r->subPos) - 7;
}
