# PNG Decoder in C

A **from-scratch PNG decoder** that implements chunk parsing, inline DEFLATE decompression, and all five PNG filter types — outputting a standard PPM file with zero external library dependencies.

Written in pure C with no external dependencies (no zlib, no libpng). Part of the Corg-Labs collection.

---

# Features
- PNG signature validation
- Full chunk parsing loop (IHDR, IDAT, IEND)
- Inline DEFLATE decompressor: stored blocks, fixed Huffman, dynamic Huffman
- Canonical Huffman table construction and decoding
- All five PNG scanline filter types: None, Sub, Up, Average, Paeth
- Supports 8-bit RGB (color type 2) and RGBA (color type 6)
- Outputs P6 binary PPM; RGBA files have the alpha channel stripped
- Error handling via `setjmp`/`longjmp` — no global abort on bad data
- Metadata (width, height, bit depth, color type) printed to stderr

---

# Tutorial

## 1. PNG File Structure

A PNG file begins with an 8-byte signature, followed by a sequence of chunks. Each chunk has: a 4-byte big-endian length, a 4-byte ASCII type, the chunk data, and a 4-byte CRC (which this decoder skips).

```
[8-byte PNG signature: 137 80 78 71 13 10 26 10]
[IHDR chunk: 4B length | "IHDR" | 13 bytes data | 4B CRC]
[IDAT chunk: 4B length | "IDAT" | N bytes zlib data | 4B CRC]
[IDAT chunk: ...may have multiple IDAT chunks concatenated...]
[IEND chunk: 4B length=0 | "IEND" | 4B CRC]
```

The decoder reads chunks in a loop until it hits `IEND`:

```c
static int read_chunk(FILE *fp, Chunk *c) {
    uint8_t hdr[8];
    if (fread(hdr, 1, 8, fp) != 8) return 0;
    c->length = read_u32be(hdr);
    memcpy(c->type, hdr + 4, 4);
    c->type[4] = '\0';
    c->data = malloc(c->length);
    fread(c->data, 1, c->length, fp);
    uint8_t crc[4];
    fread(crc, 1, 4, fp);   /* skip CRC */
    return 1;
}
```

## 2. IHDR: Image Metadata

The first chunk must be `IHDR`, 13 bytes long. It encodes width, height, bit depth, and color type.

```c
if (strcmp(c.type, "IHDR") == 0) {
    width      = read_u32be(c.data);       /* bytes 0-3 */
    height     = read_u32be(c.data + 4);   /* bytes 4-7 */
    bit_depth  = c.data[8];
    color_type = c.data[9];
    /* color_type 2 = RGB (3 bytes/pixel), 6 = RGBA (4 bytes/pixel) */
}
```

Color type 2 gives 3 channels (RGB), color type 6 gives 4 (RGBA). The decoder only supports 8-bit depths.

## 3. Collecting IDAT Data

PNG images may split the compressed data across multiple `IDAT` chunks. All `IDAT` chunks must be concatenated before decompressing.

```c
} else if (strcmp(c.type, "IDAT") == 0) {
    idat_buf = realloc(idat_buf, idat_len + c.length);
    memcpy(idat_buf + idat_len, c.data, c.length);
    idat_len += c.length;
}
```

## 4. The Bit Reader

DEFLATE reads data in bits, LSB-first within each byte. The `BitReader` struct holds a 32-bit accumulator and a count of valid bits. `br_fill` tops it up from the byte buffer; `br_read(br, n)` extracts the lowest `n` bits.

```c
typedef struct {
    const uint8_t *buf;
    size_t pos;
    uint32_t bits;
    int nbits;
} BitReader;

static uint32_t br_read(BitReader *br, int n) {
    br_fill(br);
    uint32_t v = br->bits & ((1u << n) - 1);
    br->bits >>= n;
    br->nbits -= n;
    return v;
}
```

## 5. DEFLATE Decompression

DEFLATE (RFC 1951) compresses data as a sequence of blocks. Each block starts with a `BFINAL` flag bit and a 2-bit `BTYPE`:

- `BTYPE=00` — stored (raw bytes, no compression)
- `BTYPE=01` — compressed with fixed Huffman codes
- `BTYPE=10` — compressed with dynamic Huffman codes (codes encoded in the block)

```c
while (!bfinal) {
    bfinal = (int)br_read(&br, 1);
    int btype = (int)br_read(&br, 2);

    if      (btype == 0) inflate_stored_block(&br, &out);
    else if (btype == 1) inflate_fixed_block(&br, &out);
    else if (btype == 2) inflate_dynamic_block(&br, &out);
    else DIE("reserved BTYPE");
}
```

The PNG IDAT stream is zlib-wrapped DEFLATE: skip the 2-byte zlib header (CMF + FLG) before the first bit read.

## 6. Canonical Huffman Tables

DEFLATE represents literal/length and distance values with canonical Huffman codes. A canonical Huffman table is fully described by a list of code lengths (one per symbol). `huff_build` constructs lookup arrays from this list.

```c
typedef struct {
    uint16_t sym[MAX_HUFFMAN_SYMS];   /* symbols sorted by code length */
    uint16_t count[MAX_CODE_LEN+1];   /* count of codes of each length */
    uint16_t first_code[MAX_CODE_LEN+1];
    uint16_t first_sym[MAX_CODE_LEN+1];
    int max_len;
} HuffTable;
```

Decoding reads one bit at a time, building a code value until it falls within the range for the current length:

```c
static int huff_decode(BitReader *br, const HuffTable *h) {
    uint32_t code = 0;
    for (int len = 1; len <= h->max_len; len++) {
        code = (code << 1) | br_read(br, 1);
        if (code < (uint32_t)(h->first_code[len] + h->count[len])) {
            int idx = h->first_sym[len] + (int)(code - h->first_code[len]);
            return h->sym[idx];
        }
    }
    DIE("bad huffman code");
}
```

## 7. Back-References (LZ77)

DEFLATE is LZ77 + Huffman. Symbols 257–285 in the literal/length alphabet are not literal bytes but instructions to copy a run of bytes from earlier in the output buffer.

```c
int sym = huff_decode(br, lit);
if (sym < 256) {
    dynbuf_append(&out, (uint8_t)sym);   /* literal byte */
} else if (sym == 256) {
    break;                               /* end of block */
} else {
    /* Length/distance back-reference */
    int li       = sym - 257;
    int length   = length_base[li]   + (int)br_read(br, length_extra[li]);
    int di       = huff_decode(br, dist);
    int distance = dist_base[di]     + (int)br_read(br, dist_extra[di]);
    for (int k = 0; k < length; k++)
        dynbuf_append(&out, out.data[out.len - distance]);
}
```

## 8. PNG Filter Reconstruction

After decompression, each row starts with a 1-byte filter type that describes how the raw pixel bytes were transformed before compression. Reconstruction reverses this transform to recover the original pixels.

```c
static void unfilter_scanline(int filter, uint8_t *cur, const uint8_t *prev,
                               int stride, int bpp) {
    switch (filter) {
        case 0: /* None — no-op */                                        break;
        case 1: /* Sub  — add byte to the left */
            for (int i = bpp; i < stride; i++) cur[i] += cur[i - bpp];   break;
        case 2: /* Up   — add byte from row above */
            for (int i = 0; i < stride; i++) cur[i] += prev[i];          break;
        case 3: /* Average — add (left + above) / 2 */
            for (int i = 0; i < stride; i++) {
                int a = (i >= bpp) ? cur[i - bpp] : 0;
                cur[i] += (uint8_t)((a + prev[i]) / 2);
            }                                                             break;
        case 4: /* Paeth — add paeth predictor of left, above, upper-left */
            for (int i = 0; i < stride; i++) {
                int a = (i >= bpp) ? cur[i - bpp] : 0;
                cur[i] += paeth(a, prev[i], (i >= bpp) ? prev[i - bpp] : 0);
            }                                                             break;
    }
}
```

The Paeth predictor picks whichever of the three neighbours (a=left, b=above, c=upper-left) is closest to the linear prediction `a + b - c`.

---

# Build
```
gcc pngdec.c -o pngdec
```

# Run
```
./pngdec input.png output.ppm
```

Metadata is printed to stderr; the PPM is written to the output path.

---

# Concepts Practiced
- PNG chunk format: length-type-data-CRC framing
- Big-endian 32-bit reads without relying on struct packing
- DEFLATE: stored blocks, fixed and dynamic Huffman, LZ77 back-references
- Canonical Huffman table construction from code-length arrays
- LSB-first bit stream reading with a 32-bit accumulator
- PNG adaptive filter types 0-4 and the Paeth predictor
- `setjmp`/`longjmp` for deep-call error propagation
- Growing dynamic output buffer with exponential reallocation

---

# Dependencies
Standard C libraries only: `stdio.h`, `stdlib.h`, `string.h`, `stdint.h`, `setjmp.h`
