/*
 * pngdec.c — PNG decoder (pure C, no zlib / no external libs)
 * Corg-Labs
 *
 * Implements:
 *   - PNG signature & chunk parsing
 *   - IHDR parsing (width, height, bit-depth, color type)
 *   - INFLATE / DEFLATE decompression (RFC 1951)
 *   - PNG adaptive filter reconstruction (types 0-4)
 *   - Output to raw PPM P6
 *   - Supports 8-bit RGB (color type 2) and RGBA (color type 6)
 *
 * Usage:
 *   ./pngdec input.png output.ppm
 *
 * Compile:
 *   gcc pngdec.c -o pngdec
 *
 * Metadata printed to stderr; PPM written to the output file.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <setjmp.h>

/* ── error handling ─────────────────────────────────────────────────────── */
static jmp_buf g_err_jmp;
#define DIE(msg) do { fprintf(stderr, "pngdec error: %s\n", msg); longjmp(g_err_jmp, 1); } while(0)

/* ═══════════════════════════════════════════════════════════════════════════
   BIT READER — consumes bits LSB-first from a byte buffer (DEFLATE order)
   ═══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    const uint8_t *buf;
    size_t         len;
    size_t         pos;     /* byte position */
    uint32_t       bits;    /* bit accumulator */
    int            nbits;   /* valid bits in accumulator */
} BitReader;

static void br_init(BitReader *br, const uint8_t *buf, size_t len) {
    br->buf = buf; br->len = len; br->pos = 0;
    br->bits = 0; br->nbits = 0;
}

static void br_fill(BitReader *br) {
    while (br->nbits <= 24 && br->pos < br->len) {
        br->bits |= (uint32_t)br->buf[br->pos++] << br->nbits;
        br->nbits += 8;
    }
}

static uint32_t br_read(BitReader *br, int n) {
    if (n == 0) return 0;
    br_fill(br);
    if (br->nbits < n) DIE("unexpected end of deflate stream");
    uint32_t v = br->bits & ((1u << n) - 1);
    br->bits >>= n;
    br->nbits -= n;
    return v;
}

static void br_byte_align(BitReader *br) {
    int rem = br->nbits & 7;
    if (rem) { br->bits >>= rem; br->nbits -= rem; }
}

/* ═══════════════════════════════════════════════════════════════════════════
   HUFFMAN TABLE (canonical)
   ═══════════════════════════════════════════════════════════════════════════ */
#define MAX_HUFFMAN_SYMS 288
#define MAX_CODE_LEN     16

typedef struct {
    uint16_t sym[MAX_HUFFMAN_SYMS];
    uint16_t count[MAX_CODE_LEN + 1];
    uint16_t first_code[MAX_CODE_LEN + 1];
    uint16_t first_sym [MAX_CODE_LEN + 1];
    int      max_len;
} HuffTable;

static void huff_build(HuffTable *h, const uint8_t *lengths, int nsyms) {
    memset(h->count, 0, sizeof h->count);
    h->max_len = 0;

    for (int i = 0; i < nsyms; i++) {
        h->count[lengths[i]]++;
        if (lengths[i] > h->max_len) h->max_len = lengths[i];
    }
    h->count[0] = 0;

    /* sort symbols by code length */
    uint16_t pos[MAX_CODE_LEN + 1];
    pos[0] = 0;
    for (int i = 1; i <= h->max_len; i++)
        pos[i] = pos[i-1] + h->count[i-1];

    for (int i = 0; i < nsyms; i++)
        if (lengths[i]) h->sym[pos[lengths[i]]++] = (uint16_t)i;

    /* first_sym[len] = index into sym[] for codes of that length */
    uint16_t idx = 0;
    for (int i = 1; i <= h->max_len; i++) {
        h->first_sym[i] = idx;
        idx += h->count[i];
    }

    /* first_code[len] */
    uint16_t code = 0;
    for (int i = 1; i <= h->max_len; i++) {
        h->first_code[i] = code;
        code = (uint16_t)((code + h->count[i]) << 1);
    }
}

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
    return -1;
}

/* ═══════════════════════════════════════════════════════════════════════════
   DEFLATE decompressor
   ═══════════════════════════════════════════════════════════════════════════ */

/* fixed literal/length code lengths (RFC 1951 §3.2.6) */
static void make_fixed_tables(HuffTable *lit, HuffTable *dist) {
    uint8_t ll[288], dl[32];
    int i;
    for (i =   0; i <= 143; i++) ll[i] = 8;
    for (i = 144; i <= 255; i++) ll[i] = 9;
    for (i = 256; i <= 279; i++) ll[i] = 7;
    for (i = 280; i <= 287; i++) ll[i] = 8;
    for (i = 0; i < 32; i++) dl[i] = 5;
    huff_build(lit,  ll, 288);
    huff_build(dist, dl, 32);
}

static const int length_base[29] = {
    3,4,5,6,7,8,9,10,11,13,15,17,19,23,27,31,
    35,43,51,59,67,83,99,115,131,163,195,227,258
};
static const int length_extra[29] = {
    0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5,0
};
static const int dist_base[30] = {
    1,2,3,4,5,7,9,13,17,25,33,49,65,97,129,193,
    257,385,513,769,1025,1537,2049,3073,4097,6145,8193,12289,16385,24577
};
static const int dist_extra[30] = {
    0,0,0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,9,9,10,10,11,11,12,12,13,13
};

/* Dynamic block: decode code-length Huffman codes */
static const int cl_order[19] = {
    16,17,18,0,8,7,9,6,10,5,11,4,12,3,13,2,14,1,15
};

typedef struct {
    uint8_t *data;
    size_t   cap;
    size_t   len;
} DynBuf;

static void dynbuf_append(DynBuf *b, uint8_t byte) {
    if (b->len >= b->cap) {
        b->cap = b->cap ? b->cap * 2 : 65536;
        b->data = realloc(b->data, b->cap);
        if (!b->data) DIE("out of memory");
    }
    b->data[b->len++] = byte;
}

/* Used for dynamic blocks: both lit and dist are proper Huffman tables. */
static void inflate_block_codes(BitReader *br, const HuffTable *lit,
                                 const HuffTable *dist, DynBuf *out) {
    for (;;) {
        int sym = huff_decode(br, lit);
        if (sym < 256) {
            dynbuf_append(out, (uint8_t)sym);
        } else if (sym == 256) {
            break;
        } else {
            int li = sym - 257;
            if (li < 0 || li >= 29) DIE("bad length symbol");
            int length = length_base[li] + (int)br_read(br, length_extra[li]);
            int di = huff_decode(br, dist);
            if (di < 0 || di >= 30) DIE("bad distance code");
            int distance = dist_base[di] + (int)br_read(br, dist_extra[di]);
            if ((size_t)distance > out->len) DIE("back-reference before start");
            for (int k = 0; k < length; k++) {
                dynbuf_append(out, out->data[out->len - distance]);
            }
        }
    }
}

/* Fixed Huffman block: use canonical fixed tables for both lit and dist. */
static void inflate_fixed_block(BitReader *br, DynBuf *out) {
    HuffTable lit, dist;
    make_fixed_tables(&lit, &dist);
    inflate_block_codes(br, &lit, &dist, out);
}

static void inflate_dynamic_block(BitReader *br, DynBuf *out) {
    int hlit  = (int)br_read(br, 5) + 257;
    int hdist = (int)br_read(br, 5) + 1;
    int hclen = (int)br_read(br, 4) + 4;

    uint8_t cl_lens[19] = {0};
    for (int i = 0; i < hclen; i++)
        cl_lens[cl_order[i]] = (uint8_t)br_read(br, 3);

    HuffTable cl_table;
    huff_build(&cl_table, cl_lens, 19);

    int total = hlit + hdist;
    uint8_t *all_lens = calloc((size_t)total, 1);
    if (!all_lens) DIE("out of memory");

    int i = 0;
    while (i < total) {
        int s = huff_decode(br, &cl_table);
        if (s < 16) {
            all_lens[i++] = (uint8_t)s;
        } else if (s == 16) {
            if (i == 0) DIE("repeat with no previous");
            int rep = (int)br_read(br, 2) + 3;
            while (rep-- && i < total) { all_lens[i] = all_lens[i-1]; i++; }
        } else if (s == 17) {
            int rep = (int)br_read(br, 3) + 3;
            while (rep-- && i < total) all_lens[i++] = 0;
        } else { /* 18 */
            int rep = (int)br_read(br, 7) + 11;
            while (rep-- && i < total) all_lens[i++] = 0;
        }
    }

    HuffTable lit_t, dist_t;
    huff_build(&lit_t,  all_lens,          hlit);
    huff_build(&dist_t, all_lens + hlit,   hdist);
    free(all_lens);

    inflate_block_codes(br, &lit_t, &dist_t, out);
}

/* top-level inflate (zlib wrapper: 2-byte header + adler32 trailer) */
static DynBuf inflate_zlib(const uint8_t *zdata, size_t zlen) {
    if (zlen < 2) DIE("zlib data too short");
    /* skip 2-byte zlib header (CMF, FLG) */
    BitReader br;
    br_init(&br, zdata + 2, zlen - 2);

    DynBuf out = {0};

    int bfinal = 0;
    while (!bfinal) {
        bfinal = (int)br_read(&br, 1);
        int btype = (int)br_read(&br, 2);

        if (btype == 0) {
            /* stored block — byte-align, then read LEN/NLEN through the bit
               reader so that any bytes already prefetched into the accumulator
               are consumed in the right order. */
            br_byte_align(&br);
            uint16_t len  = (uint16_t)br_read(&br, 16);
            uint16_t nlen = (uint16_t)br_read(&br, 16);
            if ((uint16_t)(len ^ nlen) != 0xFFFF) DIE("stored block len check failed");
            /* copy bytes through the bit reader to stay in sync */
            for (int k = 0; k < (int)len; k++) {
                dynbuf_append(&out, (uint8_t)br_read(&br, 8));
            }
        } else if (btype == 1) {
            inflate_fixed_block(&br, &out);
        } else if (btype == 2) {
            inflate_dynamic_block(&br, &out);
        } else {
            DIE("reserved BTYPE in deflate stream");
        }
    }
    return out;
}

/* ═══════════════════════════════════════════════════════════════════════════
   PNG chunk / IHDR / filter
   ═══════════════════════════════════════════════════════════════════════════ */
static const uint8_t PNG_SIG[8] = {137,80,78,71,13,10,26,10};

static uint32_t read_u32be(const uint8_t *p) {
    return ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3];
}

typedef struct {
    uint32_t length;
    char     type[5];
    uint8_t *data;
} Chunk;

static void free_chunk(Chunk *c) { free(c->data); }

static int read_chunk(FILE *fp, Chunk *c) {
    uint8_t hdr[8];
    if (fread(hdr, 1, 8, fp) != 8) return 0;
    c->length = read_u32be(hdr);
    memcpy(c->type, hdr+4, 4);
    c->type[4] = '\0';
    if (c->length > 0) {
        c->data = malloc(c->length);
        if (!c->data) DIE("out of memory");
        if (fread(c->data, 1, c->length, fp) != c->length) DIE("truncated chunk");
    } else {
        c->data = NULL;
    }
    uint8_t crc[4];
    fread(crc, 1, 4, fp); /* skip CRC */
    return 1;
}

/* PNG filter reconstruction (in-place, per scanline) */
static inline uint8_t paeth(int a, int b, int c) {
    int p = a + b - c;
    int pa = abs(p - a);
    int pb = abs(p - b);
    int pc = abs(p - c);
    if (pa <= pb && pa <= pc) return (uint8_t)a;
    if (pb <= pc)              return (uint8_t)b;
    return (uint8_t)c;
}

static void unfilter_scanline(int filter, uint8_t *cur, const uint8_t *prev,
                               int stride, int bpp) {
    switch (filter) {
        case 0: /* None */ break;
        case 1: /* Sub */
            for (int i = bpp; i < stride; i++)
                cur[i] += cur[i - bpp];
            break;
        case 2: /* Up */
            for (int i = 0; i < stride; i++)
                cur[i] += prev[i];
            break;
        case 3: /* Average */
            for (int i = 0; i < stride; i++) {
                int a = (i >= bpp) ? cur[i - bpp] : 0;
                int b = prev[i];
                cur[i] += (uint8_t)((a + b) / 2);
            }
            break;
        case 4: /* Paeth */
            for (int i = 0; i < stride; i++) {
                int a = (i >= bpp) ? cur[i - bpp] : 0;
                int b = prev[i];
                int c = (i >= bpp) ? prev[i - bpp] : 0;
                cur[i] += paeth(a, b, c);
            }
            break;
        default:
            DIE("unknown PNG filter type");
    }
}

/* ── main ───────────────────────────────────────────────────────────────── */
int main(int argc, char *argv[]) {
    if (argc != 3) {
        fprintf(stderr, "Usage: %s input.png output.ppm\n", argv[0]);
        return 1;
    }

    if (setjmp(g_err_jmp)) return 1;

    FILE *fp = fopen(argv[1], "rb");
    if (!fp) { fprintf(stderr, "pngdec: cannot open '%s'\n", argv[1]); return 1; }

    /* check signature */
    uint8_t sig[8];
    if (fread(sig, 1, 8, fp) != 8 || memcmp(sig, PNG_SIG, 8) != 0)
        DIE("not a valid PNG file");

    uint32_t width = 0, height = 0;
    uint8_t  bit_depth = 0, color_type = 0;
    int      got_ihdr = 0;
    int      bpp = 0;   /* bytes per pixel */
    int      channels = 0;

    /* collect all IDAT data */
    uint8_t *idat_buf = NULL;
    size_t   idat_len = 0;

    Chunk c;
    while (read_chunk(fp, &c)) {
        if (strcmp(c.type, "IHDR") == 0) {
            if (c.length < 13) DIE("IHDR too short");
            width      = read_u32be(c.data);
            height     = read_u32be(c.data + 4);
            bit_depth  = c.data[8];
            color_type = c.data[9];
            got_ihdr   = 1;

            fprintf(stderr, "PNG metadata:\n");
            fprintf(stderr, "  Width:      %u\n", width);
            fprintf(stderr, "  Height:     %u\n", height);
            fprintf(stderr, "  Bit depth:  %u\n", bit_depth);

            const char *ct_name = "unknown";
            switch (color_type) {
                case 0: ct_name = "grayscale";        channels = 1; break;
                case 2: ct_name = "RGB";               channels = 3; break;
                case 3: ct_name = "indexed (palette)"; channels = 1; break;
                case 4: ct_name = "grayscale+alpha";   channels = 2; break;
                case 6: ct_name = "RGBA";              channels = 4; break;
            }
            fprintf(stderr, "  Color type: %u (%s)\n", color_type, ct_name);

            if (bit_depth != 8)
                DIE("only 8-bit PNG supported");
            if (color_type != 2 && color_type != 6)
                DIE("only RGB (type 2) and RGBA (type 6) PNG supported");

            bpp = channels;

        } else if (strcmp(c.type, "IDAT") == 0) {
            if (!got_ihdr) DIE("IDAT before IHDR");
            idat_buf = realloc(idat_buf, idat_len + c.length);
            if (!idat_buf) DIE("out of memory");
            memcpy(idat_buf + idat_len, c.data, c.length);
            idat_len += c.length;

        } else if (strcmp(c.type, "IEND") == 0) {
            free_chunk(&c);
            break;
        }
        free_chunk(&c);
    }
    fclose(fp);

    if (!got_ihdr) DIE("no IHDR chunk found");
    if (!idat_buf) DIE("no IDAT data found");

    /* decompress IDAT (zlib-wrapped DEFLATE) */
    fprintf(stderr, "Decompressing IDAT (%zu bytes)...\n", idat_len);
    DynBuf raw = inflate_zlib(idat_buf, idat_len);
    free(idat_buf);
    fprintf(stderr, "Decompressed to %zu bytes\n", raw.len);

    /* reconstruct scanlines */
    int stride = (int)width * bpp;           /* bytes per scanline (no filter byte) */
    size_t expected = (size_t)(stride + 1) * height;
    if (raw.len < expected) DIE("decompressed data too short for image dimensions");

    uint8_t *pixels = malloc((size_t)stride * height);
    if (!pixels) DIE("out of memory");
    /* zero_row is the virtual "row above row 0" (all zeros per PNG spec) */
    uint8_t *zero_row = calloc(1, (size_t)(stride > 0 ? stride : 1));
    if (!zero_row) { free(pixels); DIE("out of memory"); }

    for (uint32_t y = 0; y < height; y++) {
        uint8_t *src      = raw.data + (size_t)y * (stride + 1);
        uint8_t  filter   = src[0];
        uint8_t *cur      = pixels + (size_t)y * stride;
        uint8_t *prev_row = (y == 0) ? zero_row
                                     : pixels + (size_t)(y - 1) * stride;
        memcpy(cur, src + 1, (size_t)stride);
        unfilter_scanline(filter, cur, prev_row, stride, bpp);
    }

    free(zero_row);
    free(raw.data);

    /* write PPM */
    FILE *out = fopen(argv[2], "wb");
    if (!out) { fprintf(stderr, "pngdec: cannot write '%s'\n", argv[2]); return 1; }
    fprintf(out, "P6\n%u %u\n255\n", width, height);

    if (color_type == 2) {
        /* RGB — write directly */
        fwrite(pixels, 1, (size_t)stride * height, out);
    } else {
        /* RGBA — drop alpha channel */
        for (uint32_t y = 0; y < height; y++) {
            uint8_t *row = pixels + (size_t)y * stride;
            for (uint32_t x = 0; x < width; x++) {
                fwrite(row + x * 4, 1, 3, out);
            }
        }
    }
    fclose(out);

    free(pixels);
    fprintf(stderr, "Wrote PPM: %s (%ux%u)\n", argv[2], width, height);
    return 0;
}
