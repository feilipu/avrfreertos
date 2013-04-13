/**
 @file		md5.c
 @brief 		support MD5 for PPPoE CHAP mode

 taken from RFC-1321/Appendix A.3
 MD5C.C - RSA Data Security, Inc., MD5 message-digest algorithm
 */

#include <string.h>

#include <avr/io.h>

#include "md5.h"

// Constants for Transform routine.
#define S11    7
#define S12   12
#define S13   17
#define S14   22
#define S21    5
#define S22    9
#define S23   14
#define S24   20
#define S31    4
#define S32   11
#define S33   16
#define S34   23
#define S41    6
#define S42   10
#define S43   15
#define S44   21

static void md5_transform (uint32_t[4], uint8_t [64]);
static void md5_encode    (uint8_t *, uint32_t *, uint32_t);
static void md5_decode    (uint32_t *, uint8_t *, uint32_t);

static uint8_t padding[64] = {
	0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0
};

// F, G, H and I are basic md5 functions.
#define F(x, y, z) (((x) & (y)) | ((~x) & (z)))
#define G(x, y, z) (((x) & (z)) | ((y) & (~z)))
#define H(x, y, z) ((x) ^ (y) ^ (z))
#define I(x, y, z) ((y) ^ ((x) | (~z)))

/**
 @brief ROTATE_LEFT rotates x left n bits.
 */
#define ROTATE_LEFT(x, n) (((x) << (n)) | ((x) >> (32-(n))))


	uint32_t FF(uint32_t a, uint32_t b, uint32_t c, uint32_t d, uint32_t x, uint32_t s, uint32_t ac)
	{
		a += F (b, c, d) + x + (uint32_t)(ac);
		a = ROTATE_LEFT (a, s);
		a += b;
		return a;
	}
	uint32_t GG(uint32_t a, uint32_t b, uint32_t c, uint32_t d, uint32_t x, uint32_t s, uint32_t ac)
	{
		a += G (b, c, d) + x + (uint32_t)(ac);
		a = ROTATE_LEFT (a, s);
		a += b;
		return a;
	}

	uint32_t HH(uint32_t a, uint32_t b, uint32_t c, uint32_t d, uint32_t x, uint32_t s, uint32_t ac)
	{
		a += H (b, c, d) + x + (uint32_t)(ac);
		a = ROTATE_LEFT (a, s);
		a += b;
		return a;
	}

	uint32_t II(uint32_t a, uint32_t b, uint32_t c, uint32_t d, uint32_t x, uint32_t s, uint32_t ac)
	{
		a += I (b, c, d) + x + (uint32_t)(ac);
		a = ROTATE_LEFT (a, s);
		a += b;
		return a;
	}

/**
 @brief	md5 initialisation. Begins an md5 operation, writing a new context.
 */
void md5_init(md5_ctx *context)
{
	context->count[0] = context->count[1] = 0;

	// Load magic initialisation constants.
	context->state[0] = 0x67452301;
	context->state[1] = 0xefcdab89;
	context->state[2] = 0x98badcfe;
	context->state[3] = 0x10325476;
}

/**
 @brief	md5 block update operation. Continues an md5 message-digest operation,
		processing another message block, and updating the context.
*/
void md5_update(md5_ctx * context, uint8_t *input, uint32_t inputLen)
{
	uint32_t i, index, partLen;

	// Compute number of bytes mod 64
	index = (uint32_t)((context->count[0] >> 3) & 0x3F);

	// Update number of bits
	if ((context->count[0] += ((uint32_t)inputLen << 3)) < ((uint32_t)inputLen << 3))
		context->count[1]++;
	context->count[1] += ((uint32_t)inputLen >> 29);

	partLen = 64 - index;

	// md5_Transform as many times as possible.
	if (inputLen >= partLen)
	{
		memcpy(&context->buffer[index], input, partLen);
		md5_transform(context->state, context->buffer);

		for (i = partLen; i + 63 < inputLen; i += 64)
			md5_transform(context->state, &input[i]);
		index = 0;
	}
	else
		i = 0;

	// Buffer remaining input
	memcpy(&context->buffer[index], &input[i], inputLen - i);
}

/**
 @brief	md5 finalisation. Ends an md5 message-digest operation, writing the
		message digest and zeroing the context.
*/
void md5_final(uint8_t digest[16], md5_ctx *context)
{
	uint8_t  bits[8];
	uint32_t index, padLen;

	// Save number of bits
	md5_encode(bits, context->count, 8);

	// Pad out to 56 mod 64.
	index  = (uint32_t)((context->count[0] >> 3) & 0x3f);
	padLen = (index < 56) ? (56 - index) : (120 - index);
	md5_update(context, padding, padLen);

	// Append length (before padding)
	md5_update(context, bits, 8);
	// Store state in digest
	md5_encode(digest, context->state, 16);

	// Zero sensitive information.
	memset((void*)context,0,sizeof(*context));
}

/**
 @brief	md5 basic transformation. Transforms state based on block.
 */
static void md5_transform(uint32_t state[4], uint8_t block[64])
{
	uint32_t a = state[0];
	uint32_t b = state[1];
	uint32_t c = state[2];
	uint32_t d = state[3];
	uint32_t x[16];

	md5_decode(x, block, 64);

	// Round 1
	a = FF(a, b, c, d, x[0],  S11, 0xd76aa478);	// 1
	d = FF(d, a, b, c, x[1],  S12, 0xe8c7b756); // 2
	c = FF(c, d, a, b, x[2],  S13, 0x242070db); // 3
	b = FF(b, c, d, a, x[3],  S14, 0xc1bdceee); // 4
	a = FF(a, b, c, d, x[4],  S11, 0xf57c0faf); // 5
	d = FF(d, a, b, c, x[5],  S12, 0x4787c62a); // 6
	c = FF(c, d, a, b, x[6],  S13, 0xa8304613); // 7
	b = FF(b, c, d, a, x[7],  S14, 0xfd469501); // 8
	a = FF(a, b, c, d, x[8],  S11, 0x698098d8); // 9
	d = FF(d, a, b, c, x[9],  S12, 0x8b44f7af); // 10
	c = FF(c, d, a, b, x[10], S13, 0xffff5bb1); // 11
	b = FF(b, c, d, a, x[11], S14, 0x895cd7be); // 12
	a = FF(a, b, c, d, x[12], S11, 0x6b901122); // 13
	d = FF(d, a, b, c, x[13], S12, 0xfd987193); // 14
	c = FF(c, d, a, b, x[14], S13, 0xa679438e); // 15
	b = FF(b, c, d, a, x[15], S14, 0x49b40821); // 16

	// Round 2
	a = GG(a, b, c, d, x[1],  S21, 0xf61e2562); // 17
	d = GG(d, a, b, c, x[6],  S22, 0xc040b340); // 18
	c = GG(c, d, a, b, x[11], S23, 0x265e5a51); // 19
	b = GG(b, c, d, a, x[0],  S24, 0xe9b6c7aa); // 20
	a = GG(a, b, c, d, x[5],  S21, 0xd62f105d); // 21
	d = GG(d, a, b, c, x[10], S22, 0x2441453);  // 22
	c = GG(c, d, a, b, x[15], S23, 0xd8a1e681); // 23
	b = GG(b, c, d, a, x[4],  S24, 0xe7d3fbc8); // 24
	a = GG(a, b, c, d, x[9],  S21, 0x21e1cde6); // 25
	d = GG(d, a, b, c, x[14], S22, 0xc33707d6); // 26
	c = GG(c, d, a, b, x[3],  S23, 0xf4d50d87); // 27
	b = GG(b, c, d, a, x[8],  S24, 0x455a14ed); // 28
	a = GG(a, b, c, d, x[13], S21, 0xa9e3e905); // 29
	d = GG(d, a, b, c, x[2],  S22, 0xfcefa3f8); // 30
	c = GG(c, d, a, b, x[7],  S23, 0x676f02d9); // 31
	b = GG(b, c, d, a, x[12], S24, 0x8d2a4c8a); // 32

	// Round 3
	a = HH(a, b, c, d, x[5],  S31, 0xfffa3942); // 33
	d = HH(d, a, b, c, x[8],  S32, 0x8771f681); // 34
	c = HH(c, d, a, b, x[11], S33, 0x6d9d6122); // 35
	b = HH(b, c, d, a, x[14], S34, 0xfde5380c); // 36
	a = HH(a, b, c, d, x[1],  S31, 0xa4beea44); // 37
	d = HH(d, a, b, c, x[4],  S32, 0x4bdecfa9); // 38
	c = HH(c, d, a, b, x[7],  S33, 0xf6bb4b60); // 39
	b = HH(b, c, d, a, x[10], S34, 0xbebfbc70); // 40
	a = HH(a, b, c, d, x[13], S31, 0x289b7ec6); // 41
	d = HH(d, a, b, c, x[0],  S32, 0xeaa127fa); // 42
	c = HH(c, d, a, b, x[3],  S33, 0xd4ef3085); // 43
	b = HH(b, c, d, a, x[6],  S34, 0x4881d05);  // 44
	a = HH(a, b, c, d, x[9],  S31, 0xd9d4d039); // 45
	d = HH(d, a, b, c, x[12], S32, 0xe6db99e5); // 46
	c = HH(c, d, a, b, x[15], S33, 0x1fa27cf8); // 47
	b = HH(b, c, d, a, x[2],  S34, 0xc4ac5665); // 48

	// Round 4
	a = II(a, b, c, d, x[0],  S41, 0xf4292244); // 49
	d = II(d, a, b, c, x[7],  S42, 0x432aff97); // 50
	c = II(c, d, a, b, x[14], S43, 0xab9423a7); // 51
	b = II(b, c, d, a, x[5],  S44, 0xfc93a039); // 52
	a = II(a, b, c, d, x[12], S41, 0x655b59c3); // 53
	d = II(d, a, b, c, x[3],  S42, 0x8f0ccc92); // 54
	c = II(c, d, a, b, x[10], S43, 0xffeff47d); // 55
	b = II(b, c, d, a, x[1],  S44, 0x85845dd1); // 56
	a = II(a, b, c, d, x[8],  S41, 0x6fa87e4f); // 57
	d = II(d, a, b, c, x[15], S42, 0xfe2ce6e0); // 58
	c = II(c, d, a, b, x[6],  S43, 0xa3014314); // 59
	b = II(b, c, d, a, x[13], S44, 0x4e0811a1); // 60
	a = II(a, b, c, d, x[4],  S41, 0xf7537e82); // 61
	d = II(d, a, b, c, x[11], S42, 0xbd3af235); // 62
	c = II(c, d, a, b, x[2],  S43, 0x2ad7d2bb); // 63
	b = II(b, c, d, a, x[9],  S44, 0xeb86d391); // 64

	state[0] += a;
	state[1] += b;
	state[2] += c;
	state[3] += d;

	// Zero sensitive information.
	memset(&x,0,sizeof(x));
}

/**
 @brief	Encodes input (uint32_t) into output (uint8_t). Assumes len is a multiple of 4.
 */
static void md5_encode(uint8_t *output, uint32_t *input, uint32_t len)
{
	uint32_t i, j;

	for (i = 0, j = 0; j < len; i++, j += 4)
	{
		output[j]     = (uint8_t)(input[i] & 0xff);
		output[j + 1] = (uint8_t)((input[i] >> 8) & 0xff);
		output[j + 2] = (uint8_t)((input[i] >> 16) & 0xff);
		output[j + 3] = (uint8_t)((input[i] >> 24) & 0xff);
	}
}

/**
 @brief Decodes input (uint8_t) into output (uint32_t). Assumes len is a multiple of 4.
 */
static void md5_decode(uint32_t *output, uint8_t *input, uint32_t len)
{
	uint32_t i, j;
	for (i = 0, j = 0; j < len; i++, j += 4)
		output[i] = ((uint32_t) input[j]) | (((uint32_t) input[j + 1]) << 8) |
				(((uint32_t)input[j + 2]) << 16) | (((uint32_t)input[j + 3]) << 24);
}

