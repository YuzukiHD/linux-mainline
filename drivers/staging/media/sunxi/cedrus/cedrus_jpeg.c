// SPDX-License-Identifier: GPL-2.0
/*
 * Cedrus VPU driver: baseline JPEG decoder of the F101 video engine.
 *
 * The engine parses the entropy coded data itself. Software reads the marker
 * segments, loads the tables, and the engine writes the picture in its tiled
 * macroblock layout. The conversion block of the engine then turns it into
 * linear NV12.
 */

#include <linux/dma-mapping.h>
#include <linux/unaligned.h>

#include <media/v4l2-mem2mem.h>

#include "cedrus.h"
#include "cedrus_hw.h"
#include "cedrus_regs.h"

/* decoder block, the one of the MPEG family */
#define JPEG_CTRL		0x114
#define JPEG_CTRL_IRQ_EN	0x7c
#define JPEG_START		0x118
#define JPEG_START_DECODE	14
#define JPEG_FORMAT		0x11b	/* byte: (chroma sampling << 3) | 3 */
#define JPEG_STATUS		0x11c	/* write 1 to clear */
#define JPEG_STATUS_DONE	BIT(0)
#define JPEG_STATUS_ALL		0x0f
#define JPEG_STREAM_ADDR	0x128
#define JPEG_STREAM_OFFSET	0x12c	/* in bits */
#define JPEG_STREAM_LEN		0x130	/* in bits */
#define JPEG_STREAM_END		0x134	/* last byte of the buffer */
#define JPEG_QTAB		0x180	/* (index << 8) | value */
#define JPEG_QTAB_FIRST		BIT(14)
#define JPEG_MCU_SIZE		0x1b8	/* ((rows - 1) << 16) | (columns - 1) */
#define JPEG_RESTART		0x1c0
#define JPEG_RECON_Y		0x1cc
#define JPEG_RECON_C		0x1d0
#define JPEG_SCALE		0x1d4
#define JPEG_HUFF_ADDR		0x1e0
#define JPEG_HUFF_DATA		0x1e4

/* top block */
#define TOP_CTRL		0x04
#define TOP_CTRL_RESET		BIT(16)
#define TOP_FLAGS		0x1c
#define TOP_REF_C_SIZE		0xc4
#define TOP_STRIDES		0xc8	/* four half words */
#define TOP_OUT_CTRL		0xe8
#define TOP_OUT_FORMAT		0xec
#define TOP_LINE_BUF_CTRL	0x50
#define TOP_LINE_BUF_ADDR	0x54

/* picture conversion block: tiled macroblocks in, linear NV12 out */
#define CV_SIZE			0xa00
#define CV_STRIDE		0xa04
#define CV_ENABLE		0xa08
#define CV_MODE			0xa0c
#define CV_STATUS		0xa10
#define CV_LINE			0xa14
#define CV_CONFIG		0xa2c
#define CV_OUT_Y		0xa70
#define CV_OUT_C		0xa74
#define CV_IN_Y			0xa78
#define CV_IN_C			0xa7c
#define CV_TOP_MODE		2	/* VE_MODE value that selects the block */

#define JPEG_HUFF_WORDS		512
#define JPEG_MAX_COMP		3

enum jpeg_sampling {
	JPEG_420 = 0,
	JPEG_411,
	JPEG_422,
	JPEG_444,
	JPEG_422T,
	JPEG_400,
	JPEG_ERR,
};

struct jpeg_huff {
	u16 start_code[16];	/* first code of each length, 0xffff when unused */
	u8 offset[16];		/* symbols of shorter codes */
	u8 symbol[256];
};

struct jpeg_comp {
	int id;
	int h, v;
	int tq;
	int td, ta;
};

struct jpeg_frame {
	int width, height;
	int ncomp;
	struct jpeg_comp comp[JPEG_MAX_COMP];
	int scan_order[JPEG_MAX_COMP];
	int max_h, max_v;
	int restart_interval;
	enum jpeg_sampling sampling;
	u16 qtab[4][64];
	struct jpeg_huff dc[2];
	struct jpeg_huff ac[2];
	bool has_dht;
	size_t data_off;
	size_t data_size;
};

/* ---- marker segments ---------------------------------------------------- */

struct reader {
	const u8 *data;
	size_t len;
	size_t pos;
};

static int rd8(struct reader *r)
{
	return r->pos < r->len ? r->data[r->pos++] : -1;
}

static int rd16(struct reader *r)
{
	int hi = rd8(r);
	int lo = rd8(r);

	return (hi < 0 || lo < 0) ? -1 : (hi << 8) | lo;
}

static enum jpeg_sampling jpeg_sampling_of(const struct jpeg_comp *comp, int ncomp)
{
	u32 key = 0;
	int i;

	if (ncomp == 1)
		return JPEG_400;
	if (ncomp != 3)
		return JPEG_ERR;
	for (i = 0; i < 3; i++)
		key = (key << 8) | (comp[i].h << 4) | comp[i].v;

	switch (key) {
	case 0x221111:
		return JPEG_420;
	case 0x411111:
		return JPEG_411;
	case 0x211111:
		return JPEG_422;
	case 0x111111:
		return JPEG_444;
	case 0x121111:
		return JPEG_422T;
	default:
		return JPEG_ERR;
	}
}

/* counts[i] codes of length i + 1 and their symbols, in the form of the engine */
static void jpeg_build_table(struct jpeg_huff *table, const u8 counts[16], const u8 *symbols)
{
	unsigned int code = 0;
	int total = 0, last = 16, i;

	memset(table, 0, sizeof(*table));
	for (i = 0; i < 16; i++) {
		table->offset[i] = total;
		table->start_code[i] = code;
		total += counts[i];
		code = (code + counts[i]) << 1;
	}
	memcpy(table->symbol, symbols, total);
	/* lengths beyond the longest code never match */
	while (last > 0 && counts[last - 1] == 0) {
		table->start_code[last - 1] = 0xffff;
		last--;
	}
}

static int jpeg_get_dqt(struct reader *r, struct jpeg_frame *frame)
{
	int len = rd16(r) - 2;

	while (len > 0) {
		int pq = rd8(r);
		int prec = pq >> 4;
		int id = pq & 15;
		int i;

		len--;
		if (pq < 0 || id > 3 || prec > 1)
			return -EINVAL;
		for (i = 0; i < 64; i++) {
			int value = prec ? rd16(r) : rd8(r);

			if (value < 0)
				return -EINVAL;
			frame->qtab[id][i] = value;
		}
		len -= prec ? 128 : 64;
	}
	return len == 0 ? 0 : -EINVAL;
}

static int jpeg_get_dht(struct reader *r, struct jpeg_frame *frame)
{
	int len = rd16(r) - 2;

	frame->has_dht = true;
	while (len > 16) {
		int tc = rd8(r);
		int id = tc & 15;
		u8 counts[16];
		u8 symbols[256];
		int total = 0, i;

		if (tc < 0 || id > 1)
			return -EINVAL;
		for (i = 0; i < 16; i++) {
			int count = rd8(r);

			if (count < 0)
				return -EINVAL;
			counts[i] = count;
			total += count;
		}
		len -= 17;
		if (total > 256 || total > len)
			return -EINVAL;
		for (i = 0; i < total; i++)
			symbols[i] = rd8(r);
		len -= total;
		jpeg_build_table((tc & 0x10) ? &frame->ac[id] : &frame->dc[id], counts, symbols);
	}
	return len == 0 ? 0 : -EINVAL;
}

static int jpeg_get_sof(struct reader *r, struct jpeg_frame *frame)
{
	int len = rd16(r);
	int prec = rd8(r);
	int i;

	frame->height = rd16(r);
	frame->width = rd16(r);
	frame->ncomp = rd8(r);
	if (len < 0 || prec != 8 || frame->height <= 0 || frame->width <= 0 || frame->ncomp < 0)
		return -EINVAL;
	if ((frame->ncomp != 1 && frame->ncomp != 3) || len != 8 + 3 * frame->ncomp)
		return -EINVAL;

	frame->max_h = frame->max_v = 0;
	for (i = 0; i < frame->ncomp; i++) {
		int id = rd8(r);
		int hv = rd8(r);
		int tq = rd8(r);

		if (id < 0 || hv < 0 || tq < 0)
			return -EINVAL;
		frame->comp[i].id = id;
		frame->comp[i].h = hv >> 4;
		frame->comp[i].v = hv & 15;
		frame->comp[i].tq = tq & 3;
		if (!frame->comp[i].h || !frame->comp[i].v)
			return -EINVAL;
		frame->max_h = max(frame->max_h, frame->comp[i].h);
		frame->max_v = max(frame->max_v, frame->comp[i].v);
	}
	frame->sampling = jpeg_sampling_of(frame->comp, frame->ncomp);

	return frame->sampling == JPEG_ERR ? -EINVAL : 0;
}

static int jpeg_get_sos(struct reader *r, struct jpeg_frame *frame)
{
	int len = rd16(r);
	int count = rd8(r);
	int i;

	if (!frame->ncomp || count != frame->ncomp || len != 6 + 2 * count)
		return -EINVAL;
	for (i = 0; i < count; i++) {
		int id = rd8(r);
		int tt = rd8(r);
		int idx;

		if (id < 0 || tt < 0)
			return -EINVAL;
		for (idx = 0; idx < frame->ncomp; idx++)
			if (frame->comp[idx].id == id)
				break;
		if (idx == frame->ncomp)
			return -EINVAL;
		frame->comp[idx].td = (tt >> 4) & 1;
		frame->comp[idx].ta = tt & 1;
		frame->scan_order[i] = id;
	}
	/* spectral selection and successive approximation: baseline has 0, 63, 0 */
	if (rd8(r) != 0 || rd8(r) != 63 || rd8(r) != 0)
		return -EINVAL;
	return 0;
}

/* Reads the segments of one picture up to the start of the scan */
static int jpeg_parse(const u8 *buf, size_t len, struct jpeg_frame *frame)
{
	struct reader r = { .data = buf, .len = len };
	bool sof = false;

	memset(frame, 0, sizeof(*frame));
	if (len < 4 || buf[0] != 0xff || buf[1] != 0xd8)
		return -EINVAL;
	r.pos = 2;

	for (;;) {
		int marker = rd8(&r);
		int ret = 0;

		if (marker < 0)
			return -EINVAL;
		if (marker != 0xff)
			continue;	/* damaged: look for the next marker */
		while ((marker = rd8(&r)) == 0xff)
			;
		if (marker < 0)
			return -EINVAL;

		switch (marker) {
		case 0:
		case 1:
		case 0xd8:
			break;
		case 0xc0:
		case 0xc1:
			ret = jpeg_get_sof(&r, frame);
			sof = ret == 0;
			break;
		case 0xc2:	/* progressive, lossless, arithmetic: not supported */
		case 0xc3:
		case 0xc5 ... 0xc7:
		case 0xc9 ... 0xcf:
			return -EINVAL;
		case 0xc4:
			ret = jpeg_get_dht(&r, frame);
			break;
		case 0xdb:
			ret = jpeg_get_dqt(&r, frame);
			break;
		case 0xdd:
			if (rd16(&r) != 4)
				return -EINVAL;
			frame->restart_interval = rd16(&r);
			break;
		case 0xda:
			if (!sof)
				return -EINVAL;
			ret = jpeg_get_sos(&r, frame);
			if (!ret) {
				size_t end = len;

				frame->data_off = r.pos;
				/* the data ends with the last EOI, which the engine needs too */
				while (end > frame->data_off + 1 &&
				       !(buf[end - 2] == 0xff && buf[end - 1] == 0xd9))
					end--;
				frame->data_size = end - frame->data_off;
			}
			return ret;
		case 0xd9:
			return -EINVAL;
		default:
			if (marker >= 0xd0 && marker <= 0xd7)
				break;
			/* APPn, COM and the rest: skip the segment */
			{
				int seg = rd16(&r);

				if (seg < 2 || r.pos + seg - 2 > len)
					return -EINVAL;
				r.pos += seg - 2;
			}
			break;
		}
		if (ret)
			return ret;
	}
}

/* ---- table images --------------------------------------------------------- */

/*
 * Table memory of the engine: 1024 half words, four 128-entry blocks of
 * start codes / offsets / zero padding for DC0 AC0 DC1 AC1, then the symbols
 * in four 128-entry blocks starting at 512.
 */
static void jpeg_put_table(u16 *img, int block, const struct jpeg_huff *table)
{
	u16 *words = img + block * 32;
	int i;

	memcpy(words, table->start_code, sizeof(table->start_code));
	for (i = 0; i < 8; i++) {
		words[16 + i] = table->offset[2 * i] | (table->offset[2 * i + 1] << 8);
		words[24 + i] = 0;
	}
	for (i = 0; i < 128; i++)
		img[512 + block * 128 + i] = table->symbol[2 * i] | (table->symbol[2 * i + 1] << 8);
}

static const struct jpeg_comp *jpeg_scan_comp(const struct jpeg_frame *frame, int index)
{
	int id = frame->scan_order[index < frame->ncomp ? index : 0];
	int i;

	for (i = 0; i < frame->ncomp; i++)
		if (frame->comp[i].id == id)
			return &frame->comp[i];
	return &frame->comp[0];
}

static void jpeg_huff_image(const struct jpeg_frame *frame, u16 img[2 * JPEG_HUFF_WORDS])
{
	const struct jpeg_comp *c0 = jpeg_scan_comp(frame, 0);
	const struct jpeg_comp *c1 = jpeg_scan_comp(frame, 1);

	memset(img, 0, 2 * JPEG_HUFF_WORDS * sizeof(u16));
	jpeg_put_table(img, 0, &frame->dc[c0->td]);
	jpeg_put_table(img, 1, &frame->ac[c0->ta]);
	jpeg_put_table(img, 2, &frame->dc[c1->td]);
	jpeg_put_table(img, 3, &frame->ac[c1->ta]);
}

/* The standard tables of the baseline process, used when a stream has none */
static const u8 std_dc_l_counts[16] = { 0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0 };
static const u8 std_dc_c_counts[16] = { 0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0 };
static const u8 std_dc_symbols[12] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };
static const u8 std_ac_l_counts[16] = { 0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d };
static const u8 std_ac_l_symbols[162] = {
	0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61,
	0x07, 0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08, 0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52,
	0xd1, 0xf0, 0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x25,
	0x26, 0x27, 0x28, 0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45,
	0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64,
	0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x83,
	0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99,
	0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6,
	0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3,
	0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8,
	0xe9, 0xea, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa,
};
static const u8 std_ac_c_counts[16] = { 0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77 };
static const u8 std_ac_c_symbols[162] = {
	0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61,
	0x71, 0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91, 0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33,
	0x52, 0xf0, 0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34, 0xe1, 0x25, 0xf1, 0x17, 0x18,
	0x19, 0x1a, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44,
	0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63,
	0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a,
	0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97,
	0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4,
	0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca,
	0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7,
	0xe8, 0xe9, 0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa,
};


static void jpeg_default_huff_image(u16 img[2 * JPEG_HUFF_WORDS])
{
	struct jpeg_frame *frame;
	int i;

	frame = kzalloc(sizeof(*frame), GFP_KERNEL);
	if (!frame) {
		memset(img, 0, 2 * JPEG_HUFF_WORDS * sizeof(u16));
		return;
	}
	frame->ncomp = 3;
	jpeg_build_table(&frame->dc[0], std_dc_l_counts, std_dc_symbols);
	jpeg_build_table(&frame->dc[1], std_dc_c_counts, std_dc_symbols);
	jpeg_build_table(&frame->ac[0], std_ac_l_counts, std_ac_l_symbols);
	jpeg_build_table(&frame->ac[1], std_ac_c_counts, std_ac_c_symbols);
	for (i = 0; i < 3; i++) {
		frame->comp[i].id = i;
		frame->scan_order[i] = i;
		frame->comp[i].td = frame->comp[i].ta = i > 0;
	}
	jpeg_huff_image(frame, img);
	kfree(frame);
}

/* ---- hardware ------------------------------------------------------------- */

static const u16 *jpeg_qtab_of(const struct jpeg_frame *frame, int index)
{
	int id = frame->scan_order[index < frame->ncomp ? index : 0];
	int i;

	for (i = 0; i < frame->ncomp; i++)
		if (frame->comp[i].id == id)
			return frame->qtab[frame->comp[i].tq];
	return frame->qtab[0];
}

static u32 jpeg_stream_addr_word(dma_addr_t phys)
{
	return (phys >> 28) | (phys & 0x0ffffff0) | 0x70000000;
}

/* The tiled picture of a size the conversion block takes: macroblock aligned sizes */
static size_t jpeg_tile_chroma_size(unsigned int width, unsigned int height)
{
	return 2 * ALIGN((width + 1) >> 1, 16) * ALIGN((height + 1) >> 1, 32);
}

static void jpeg_hw_reset(struct cedrus_dev *dev)
{
	cedrus_write(dev, TOP_CTRL, cedrus_read(dev, TOP_CTRL) | TOP_CTRL_RESET);
	cedrus_write(dev, TOP_CTRL, cedrus_read(dev, TOP_CTRL) & ~TOP_CTRL_RESET);
	cedrus_write(dev, TOP_FLAGS, cedrus_read(dev, TOP_FLAGS) & ~(BIT(8) | BIT(10)));
	/* output side registers back to their idle values */
	cedrus_write(dev, TOP_REF_C_SIZE, 0);
	writew(0, dev->base + TOP_STRIDES);
	writew(0, dev->base + TOP_STRIDES + 2);
	writew(0, dev->base + TOP_STRIDES + 4);
	writew(0, dev->base + TOP_STRIDES + 6);
	cedrus_write(dev, TOP_OUT_CTRL, 0);
	cedrus_write(dev, TOP_OUT_FORMAT, cedrus_read(dev, TOP_OUT_FORMAT) & ~0x77);
	cedrus_write(dev, TOP_LINE_BUF_CTRL, cedrus_read(dev, TOP_LINE_BUF_CTRL) & ~0xf);
	cedrus_write(dev, TOP_LINE_BUF_ADDR, 0);
}

static int cedrus_jpeg_start(struct cedrus_ctx *ctx)
{
	unsigned int width = ALIGN(ctx->src_fmt.width, 32);
	unsigned int height = ALIGN(ctx->src_fmt.height, 32);

	ctx->codec.jpeg.tile_chroma = width * height;
	ctx->codec.jpeg.tile_size = width * height + jpeg_tile_chroma_size(ctx->src_fmt.width,
									   ctx->src_fmt.height);
	ctx->codec.jpeg.tile = dma_alloc_coherent(ctx->dev->dev, ctx->codec.jpeg.tile_size,
						  &ctx->codec.jpeg.tile_dma, GFP_KERNEL);
	if (!ctx->codec.jpeg.tile)
		return -ENOMEM;

	jpeg_hw_reset(ctx->dev);
	return 0;
}

static void cedrus_jpeg_stop(struct cedrus_ctx *ctx)
{
	dma_free_coherent(ctx->dev->dev, ctx->codec.jpeg.tile_size, ctx->codec.jpeg.tile,
			  ctx->codec.jpeg.tile_dma);
	ctx->codec.jpeg.tile = NULL;
}

static int cedrus_jpeg_setup(struct cedrus_ctx *ctx, struct cedrus_run *run)
{
	struct cedrus_dev *dev = ctx->dev;
	struct vb2_buffer *src = &run->src->vb2_buf;
	struct jpeg_frame *frame;
	dma_addr_t stream = vb2_dma_contig_plane_dma_addr(src, 0);
	const u16 *q0, *q1;
	u16 *huff;
	int mcu_w, mcu_h, i, ret;

	frame = kzalloc(sizeof(*frame), GFP_KERNEL);
	huff = kmalloc_array(2 * JPEG_HUFF_WORDS, sizeof(*huff), GFP_KERNEL);
	if (!frame || !huff) {
		ret = -ENOMEM;
		goto out;
	}

	ret = jpeg_parse(vb2_plane_vaddr(src, 0), vb2_get_plane_payload(src, 0), frame);
	if (ret)
		goto out;

	/*
	 * Only 4:2:0 pictures are made: the other layouts (and gray, which decodes
	 * to a wrong luma plane) come out in tiled formats of their own.
	 */
	if (frame->sampling != JPEG_420 || ALIGN(frame->width, 32) != ctx->dst_fmt.width ||
	    ALIGN(frame->height, 32) != ctx->dst_fmt.height) {
		ret = -EINVAL;
		goto out;
	}
	ctx->codec.jpeg.width = ctx->dst_fmt.width;
	ctx->codec.jpeg.height = ctx->dst_fmt.height;

	if (frame->has_dht)
		jpeg_huff_image(frame, huff);
	else
		jpeg_default_huff_image(huff);

	cedrus_engine_enable(ctx);

	q0 = jpeg_qtab_of(frame, 0);
	q1 = jpeg_qtab_of(frame, 1);

	cedrus_write(dev, JPEG_CTRL, 0);
	cedrus_write(dev, JPEG_RESTART, frame->restart_interval);
	writeb((frame->sampling << 3) | 3, dev->base + JPEG_FORMAT);
	for (i = 0; i < 64; i++)
		cedrus_write(dev, JPEG_QTAB, (i << 8) | q0[i] | JPEG_QTAB_FIRST);
	for (i = 0; i < 64; i++)
		cedrus_write(dev, JPEG_QTAB, (i << 8) | q1[i]);
	cedrus_write(dev, JPEG_RECON_Y, ctx->codec.jpeg.tile_dma);
	cedrus_write(dev, JPEG_RECON_C, ctx->codec.jpeg.tile_dma + ctx->codec.jpeg.tile_chroma);
	mcu_w = DIV_ROUND_UP(frame->width, frame->max_h * 8);
	mcu_h = DIV_ROUND_UP(frame->height, frame->max_v * 8);
	cedrus_write(dev, JPEG_MCU_SIZE, ((mcu_h - 1) << 16) | ((mcu_w - 1) & 0xffff));
	cedrus_write(dev, JPEG_SCALE, 0);

	cedrus_write(dev, JPEG_HUFF_ADDR, 0);
	for (i = 0; i < JPEG_HUFF_WORDS; i++)
		cedrus_write(dev, JPEG_HUFF_DATA, ((u32 *)huff)[i]);

	cedrus_write(dev, JPEG_STREAM_END, stream + vb2_plane_size(src, 0) - 1);
	cedrus_write(dev, JPEG_CTRL, cedrus_read(dev, JPEG_CTRL) | JPEG_CTRL_IRQ_EN);
	cedrus_write(dev, JPEG_STREAM_OFFSET, frame->data_off << 3);
	cedrus_write(dev, JPEG_STREAM_LEN, frame->data_size << 3);
	cedrus_write(dev, JPEG_STREAM_ADDR, jpeg_stream_addr_word(stream));

	ctx->codec.jpeg.out = cedrus_dst_buf_addr(ctx, &run->dst->vb2_buf, 0);
	ctx->codec.jpeg.converting = false;
	ret = 0;
out:
	kfree(huff);
	kfree(frame);
	return ret;
}

static void cedrus_jpeg_trigger(struct cedrus_ctx *ctx)
{
	writeb(JPEG_START_DECODE, ctx->dev->base + JPEG_START);
}

/* Converts the tiled picture to linear NV12; the chroma plane follows the luma plane */
static void cedrus_jpeg_convert_start(struct cedrus_ctx *ctx)
{
	struct cedrus_dev *dev = ctx->dev;
	u32 width = ctx->codec.jpeg.width;
	u32 height = ctx->codec.jpeg.height;
	u32 mbw = width / 16;
	u32 mbh = height / 16;

	cedrus_write(dev, TOP_CTRL, 1);
	cedrus_write(dev, TOP_CTRL, 0);
	cedrus_write(dev, VE_MODE, CV_TOP_MODE);
	cedrus_write(dev, CV_STATUS, 1);
	cedrus_write(dev, CV_SIZE, (mbw << 16) | mbh);
	cedrus_write(dev, CV_STRIDE, (mbw << 16) | mbw);
	cedrus_write(dev, CV_LINE, mbw);
	cedrus_write(dev, CV_ENABLE, 0x50080001);
	cedrus_write(dev, CV_CONFIG, (mbh << 16) | mbw);
	cedrus_write(dev, 0xa30, 0);
	cedrus_write(dev, 0xa34, 0);
	cedrus_write(dev, 0xa38, 0);
	cedrus_write(dev, CV_OUT_Y, ctx->codec.jpeg.out);
	cedrus_write(dev, CV_OUT_C, ctx->codec.jpeg.out + width * height);
	cedrus_write(dev, CV_IN_Y, ctx->codec.jpeg.tile_dma);
	cedrus_write(dev, CV_IN_C, ctx->codec.jpeg.tile_dma + ctx->codec.jpeg.tile_chroma);
	cedrus_write(dev, CV_MODE, 1);
}

/*
 * A job has two parts, each ending with an interrupt: the decoder writes the
 * tiled picture, then the conversion block makes linear NV12 of it.
 */
static enum cedrus_irq_status cedrus_jpeg_irq_status(struct cedrus_ctx *ctx)
{
	struct cedrus_dev *dev = ctx->dev;

	if (ctx->codec.jpeg.converting) {
		if (!(cedrus_read(dev, CV_ENABLE) & 1) || !(cedrus_read(dev, CV_STATUS) & 1))
			return CEDRUS_IRQ_NONE;
		return CEDRUS_IRQ_OK;
	}

	if (!(cedrus_read(dev, JPEG_CTRL) & JPEG_CTRL_IRQ_EN) ||
	    !(cedrus_read(dev, JPEG_STATUS) & JPEG_STATUS_ALL))
		return CEDRUS_IRQ_NONE;

	cedrus_write(dev, JPEG_CTRL, cedrus_read(dev, JPEG_CTRL) & ~(JPEG_CTRL_IRQ_EN | 1));
	cedrus_write(dev, JPEG_STATUS, cedrus_read(dev, JPEG_STATUS) | JPEG_STATUS_ALL);
	ctx->codec.jpeg.converting = true;
	cedrus_jpeg_convert_start(ctx);
	return CEDRUS_IRQ_CONTINUE;
}

static void cedrus_jpeg_irq_clear(struct cedrus_ctx *ctx)
{
	struct cedrus_dev *dev = ctx->dev;

	cedrus_write(dev, CV_ENABLE, cedrus_read(dev, CV_ENABLE) & ~1);
	cedrus_write(dev, CV_STATUS, 1);
}

static void cedrus_jpeg_irq_disable(struct cedrus_ctx *ctx)
{
}

struct cedrus_dec_ops cedrus_dec_ops_jpeg = {
	.irq_clear	= cedrus_jpeg_irq_clear,
	.irq_disable	= cedrus_jpeg_irq_disable,
	.irq_status	= cedrus_jpeg_irq_status,
	.setup		= cedrus_jpeg_setup,
	.start		= cedrus_jpeg_start,
	.stop		= cedrus_jpeg_stop,
	.trigger	= cedrus_jpeg_trigger,
};
