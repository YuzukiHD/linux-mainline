// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Allwinner F101 G2D memory to memory device.
 *
 * The engine converts a YUV or RGB picture to RGB, scales it, and rotates or
 * flips RGB pictures. Every frame becomes a register command list that the
 * hardware reads from memory; the task end interrupt completes the frame.
 *
 * The pixel path is a small pipeline:
 *
 *   V0 (scalable, YUV)  -> [video scaler] -> blender -> write back
 *
 * The rotation is a separate DMA to DMA engine that cannot scale or convert.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "sun252i-g2d.h"

#define G2D_NAME		"sun252i-g2d"
#define G2D_MIN_SIZE		16
#define G2D_MAX_SIZE		8192
#define G2D_PITCH_ALIGN		16
#define G2D_MODULE_RATE		300000000UL
#define G2D_TIMEOUT_MS		1000
#define G2D_THREAD_ID		0

/* the engine cache and bandwidth limit fields, as the vendor sets them */
#define G2D_CACHE_SIZE_VALUE	0x40
#define G2D_DDR_LIMIT_VALUE	0x90

/* register command list */
#define G2D_CL_MAX_BLOCKS	12
#define G2D_CL_ALIGN		32
#define G2D_CL_HEADER_BYTES	(G2D_CL_MAX_BLOCKS * 16)
#define G2D_CL_BYTES		4096
#define G2D_CL_DIRTY		BIT(0)

/* overlay alpha modes */
#define OVL_ALPHA_PIXEL		0

/* a single layer in pipe 0, passed through unchanged */
#define BLD_PIPE0_ONLY		0x03010301
#define BLD_ROP_BYPASS		0xf0
#define BLD_ROP_INDEX0_DEFAULT	0x41000
#define BLD_FILL_YUV_BLACK	0x00108080

#define VSU_FRAC_BITS		19
#define VSU_COARSE_MAX_W	2048
#define ROT_MODE_NORMAL		1

/* index of the colour space converter that handles the output of the blender */
#define BLD_CSC_OUTPUT		2

struct g2d_fmt {
	u32 fourcc;
	u8 hw;
	u8 planes;
	bool yuv;
	/* bytes per sample of each plane, a UV pair counts as one sample */
	u8 bytes[3];
	/* log2 of the subsampling of each plane against the luma plane */
	u8 hshift[3];
	u8 vshift[3];
	/* the write back can produce RGB only */
	bool dst;
};

#define RGB_FMT(_fourcc, _hw, _bytes) \
	{ .fourcc = (_fourcc), .hw = (_hw), .planes = 1, .bytes = { (_bytes) }, .dst = true }

static const struct g2d_fmt g2d_formats[] = {
	RGB_FMT(V4L2_PIX_FMT_ABGR32, 0x00, 4),
	RGB_FMT(V4L2_PIX_FMT_RGBA32, 0x01, 4),
	RGB_FMT(V4L2_PIX_FMT_BGRA32, 0x02, 4),
	RGB_FMT(V4L2_PIX_FMT_ARGB32, 0x03, 4),
	RGB_FMT(V4L2_PIX_FMT_XBGR32, 0x04, 4),
	RGB_FMT(V4L2_PIX_FMT_RGBX32, 0x05, 4),
	RGB_FMT(V4L2_PIX_FMT_BGR24, 0x08, 3),
	RGB_FMT(V4L2_PIX_FMT_RGB24, 0x09, 3),
	RGB_FMT(V4L2_PIX_FMT_RGB565, 0x0a, 2),
	{
		.fourcc = V4L2_PIX_FMT_NV12, .hw = 0x28, .planes = 2, .yuv = true,
		.bytes = { 1, 2 }, .hshift = { 0, 1 }, .vshift = { 0, 1 },
	},
	{
		.fourcc = V4L2_PIX_FMT_NV21, .hw = 0x29, .planes = 2, .yuv = true,
		.bytes = { 1, 2 }, .hshift = { 0, 1 }, .vshift = { 0, 1 },
	},
	{
		.fourcc = V4L2_PIX_FMT_YUV420, .hw = 0x2a, .planes = 3, .yuv = true,
		.bytes = { 1, 1, 1 }, .hshift = { 0, 1, 1 }, .vshift = { 0, 1, 1 },
	},
	{
		.fourcc = V4L2_PIX_FMT_YUYV, .hw = 0x20, .planes = 1, .yuv = true,
		.bytes = { 2 },
	},
};

struct g2d_dev {
	struct v4l2_device v4l2_dev;
	struct video_device vfd;
	struct v4l2_m2m_dev *m2m_dev;
	struct device *dev;

	struct mutex mutex;		/* serializes the ioctls */
	spinlock_t lock;		/* the running job */
	struct g2d_ctx *running;
	struct delayed_work watchdog;

	void __iomem *regs;
	struct clk *clk_mod;
	struct clk *clk_bus;
	struct clk *clk_mbus;
	struct reset_control *rst;

	void *cl_cpu;
	dma_addr_t cl_dma;
};

struct g2d_ctx {
	struct v4l2_fh fh;
	struct g2d_dev *dev;
	struct v4l2_pix_format src;
	struct v4l2_pix_format dst;
	const struct g2d_fmt *src_fmt;
	const struct g2d_fmt *dst_fmt;
	struct v4l2_ctrl_handler ctrl_handler;
	int rotate;
	bool hflip;
	bool vflip;
	u32 sequence_out;
	u32 sequence_cap;
};

static inline struct g2d_ctx *file2ctx(struct file *file)
{
	return container_of(file->private_data, struct g2d_ctx, fh);
}

/* a picture as the DMA engines see it */
struct g2d_hw_buf {
	const struct g2d_fmt *fmt;
	u32 addr[3];
	u32 pitch[3];
	u32 width;
	u32 height;
};

/* ---- formats ------------------------------------------------------------ */

static const struct g2d_fmt *g2d_find_fmt(u32 fourcc, bool capture)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(g2d_formats); i++) {
		if (g2d_formats[i].fourcc != fourcc)
			continue;
		if (capture && !g2d_formats[i].dst)
			return NULL;
		return &g2d_formats[i];
	}
	return NULL;
}

static u32 g2d_plane_pitch(const struct g2d_fmt *fmt, unsigned int plane, u32 luma_pitch)
{
	if (plane == 0 || fmt->planes == 2)
		return luma_pitch;
	return luma_pitch >> fmt->hshift[plane];
}

static u32 g2d_plane_size(const struct g2d_fmt *fmt, unsigned int plane, u32 luma_pitch, u32 height)
{
	return g2d_plane_pitch(fmt, plane, luma_pitch) * (height >> fmt->vshift[plane]);
}

static void g2d_fill_pix(struct v4l2_pix_format *pix, const struct g2d_fmt *fmt)
{
	unsigned int plane;
	u32 min_pitch;

	pix->pixelformat = fmt->fourcc;
	pix->width = clamp(round_up(pix->width, 2), G2D_MIN_SIZE, G2D_MAX_SIZE);
	pix->height = clamp(round_up(pix->height, 2), G2D_MIN_SIZE, G2D_MAX_SIZE);
	pix->field = V4L2_FIELD_NONE;

	min_pitch = pix->width * fmt->bytes[0];
	pix->bytesperline = max(round_up(pix->bytesperline, G2D_PITCH_ALIGN),
				round_up(min_pitch, G2D_PITCH_ALIGN));
	pix->sizeimage = 0;
	for (plane = 0; plane < fmt->planes; plane++)
		pix->sizeimage += g2d_plane_size(fmt, plane, pix->bytesperline, pix->height);
}

static void g2d_fill_colorspace(struct g2d_ctx *ctx)
{
	/* the picture keeps the colour space of the source, the output is RGB */
	ctx->dst.colorspace = ctx->src.colorspace;
	ctx->dst.xfer_func = ctx->src.xfer_func;
	ctx->dst.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	ctx->dst.quantization = V4L2_QUANTIZATION_FULL_RANGE;
}

/* ---- register command list ---------------------------------------------- */

struct g2d_cmdlist {
	u8 *mem;
	dma_addr_t dma;
	size_t used;
	unsigned int blocks;
};

struct g2d_rcq_header {
	__le32 addr_low;
	__le32 len;
	__le32 flags;
	__le32 reg_offset;
};

static void cl_init(struct g2d_cmdlist *cl, void *mem, dma_addr_t dma)
{
	cl->mem = mem;
	cl->dma = dma;
	cl->used = G2D_CL_HEADER_BYTES;
	cl->blocks = 0;
	memset(mem, 0, G2D_CL_HEADER_BYTES);
}

/* reserve a zeroed register image that the hardware copies to @offset */
static __le32 *cl_block(struct g2d_cmdlist *cl, u32 offset, size_t bytes)
{
	struct g2d_rcq_header *hdr = (struct g2d_rcq_header *)cl->mem + cl->blocks;
	size_t span = round_up(bytes, G2D_CL_ALIGN);
	u8 *data;

	if (cl->blocks >= G2D_CL_MAX_BLOCKS || cl->used + span > G2D_CL_BYTES)
		return NULL;

	data = cl->mem + cl->used;
	memset(data, 0, span);
	hdr->addr_low = cpu_to_le32(cl->dma + cl->used);
	hdr->len = cpu_to_le32(bytes);
	hdr->flags = cpu_to_le32(G2D_CL_DIRTY);
	hdr->reg_offset = cpu_to_le32(offset);

	cl->used += span;
	cl->blocks++;
	return (__le32 *)data;
}

/* the hardware reads the headers in pairs; unused entries are not dirty */
static size_t cl_finish(struct g2d_cmdlist *cl)
{
	return round_up(cl->blocks, 2) * sizeof(struct g2d_rcq_header);
}

static void put(__le32 *block, u32 offset, u32 value)
{
	block[offset / 4] = cpu_to_le32(value);
}

static void put_or(__le32 *block, u32 offset, u32 value)
{
	block[offset / 4] |= cpu_to_le32(value);
}

static void put_table(__le32 *block, u32 offset, const u32 *table, unsigned int count)
{
	unsigned int i;

	for (i = 0; i < count; i++)
		block[offset / 4 + i] = cpu_to_le32(table[i]);
}

/* ---- compose: a frame -> register images -------------------------------- */

struct g2d_mixer {
	struct g2d_cmdlist *cl;
	__le32 *core;
	__le32 *wb;
	__le32 *v0;
	__le32 *bld;
};

static int mixer_begin(struct g2d_mixer *mix, struct g2d_cmdlist *cl)
{
	unsigned int i;

	mix->cl = cl;
	mix->core = cl_block(cl, G2D_CORE_CTRL, G2D_CORE_CTRL_SIZE);
	mix->wb = cl_block(cl, G2D_WB, G2D_WB_SIZE);
	mix->v0 = cl_block(cl, G2D_V0, G2D_V0_SIZE);
	/* the unused UI overlays are written as disabled */
	for (i = 0; i < 3; i++)
		if (!cl_block(cl, G2D_UI(i), G2D_UI_SIZE))
			return -ENOMEM;
	mix->bld = cl_block(cl, G2D_BLD, G2D_BLD_SIZE);
	if (!mix->core || !mix->wb || !mix->v0 || !mix->bld)
		return -ENOMEM;

	put(mix->core, 0, G2D_CORE_CTRL_MIXER);
	/* the blender passes the blend result through, no raster operation */
	put(mix->bld, G2D_BLD_ROP_CTRL, BLD_ROP_BYPASS);
	put(mix->bld, G2D_BLD_ROP_INDEX0, BLD_ROP_INDEX0_DEFAULT);
	return 0;
}

static void v0_setup(struct g2d_mixer *mix, const struct g2d_hw_buf *buf)
{
	unsigned int plane;

	put(mix->v0, G2D_OVL_ATTR, G2D_OVL_ATTR_EN |
	    FIELD_PREP(G2D_OVL_ATTR_ALPHA_MODE_MASK, OVL_ALPHA_PIXEL) |
	    FIELD_PREP(G2D_OVL_ATTR_FORMAT_MASK, buf->fmt->hw) |
	    FIELD_PREP(G2D_OVL_ATTR_ALPHA_MASK, 0xff));
	put(mix->v0, G2D_OVL_MEM_SIZE, G2D_SIZE(buf->width, buf->height));
	put(mix->v0, G2D_V0_WIN_SIZE, G2D_SIZE(buf->width, buf->height));
	put(mix->v0, G2D_OVL_PITCH0, buf->pitch[0]);
	put(mix->v0, G2D_V0_PITCH1, buf->pitch[1]);
	put(mix->v0, G2D_V0_PITCH2, buf->pitch[2]);
	for (plane = 0; plane < 3; plane++)
		put(mix->v0, G2D_V0_ADDR(plane), buf->addr[plane]);
}

/*
 * Coarse down sampling in front of the scaler: the scaler itself handles up
 * to 8x horizontally and 4x vertically, larger reductions are pre-reduced by
 * the overlay. Returns the size the scaler sees.
 */
static void v0_coarse(struct g2d_mixer *mix, const struct g2d_fmt *fmt, u32 in_w, u32 in_h,
		      u32 out_w, u32 out_h, u32 *mid_w, u32 *mid_h)
{
	unsigned int hs = fmt->planes > 1 ? fmt->hshift[1] : 0;
	unsigned int vs = fmt->planes > 1 ? fmt->vshift[1] : 0;
	bool hor = false, ver = false;

	*mid_w = in_w;
	*mid_h = in_h;

	/* packed YUV has no coarse stage */
	if (fmt->planes == 1 && fmt->yuv)
		return;

	if (in_w >= (out_w << 3)) {
		*mid_w = min(out_w << 3, VSU_COARSE_MAX_W);
		hor = true;
	} else if (in_w > VSU_COARSE_MAX_W) {
		*mid_w = VSU_COARSE_MAX_W;
		hor = true;
	}
	if (in_h >= (out_h << 2)) {
		*mid_h = out_h << 2;
		ver = true;
	}

	if (hor) {
		put(mix->v0, G2D_V0_HDS0, (*mid_w << 16) | in_w);
		put(mix->v0, G2D_V0_HDS1, ((*mid_w >> hs) << 16) | ((in_w + BIT(hs) - 1) >> hs));
	}
	if (ver) {
		put(mix->v0, G2D_V0_VDS0, (*mid_h << 16) | in_h);
		put(mix->v0, G2D_V0_VDS1, ((*mid_h >> vs) << 16) | ((in_h + BIT(vs) - 1) >> vs));
	}
}

/* coefficient set for a scale step (1/8 resolution of the ratio) */
static unsigned int vsu_coef_set(u32 step)
{
	u32 ratio = step >> (VSU_FRAC_BITS - 3);
	u32 ip = ratio >> 3;
	u32 fp = ratio & 7;

	switch (ip) {
	case 0:
		return 0;
	case 1:
		return 1 + fp;
	case 2:
		return 9 + (fp >> 1);
	case 3:
		return 13;
	case 4:
		return 14;
	default:
		return 15;
	}
}

/* enable the video scaler for the V0 overlay: in (after the coarse stage) -> out */
static int vsu_enable(struct g2d_mixer *mix, const struct g2d_fmt *fmt, u32 in_w, u32 in_h, u32 out_w, u32 out_h)
{
	__le32 *vsu = cl_block(mix->cl, G2D_VSU, G2D_VSU_SIZE);
	bool planar420 = fmt->planes > 1;
	bool packed422 = fmt->yuv && fmt->planes == 1;
	u32 hstep, vstep, cw, ch;

	if (!vsu)
		return -ENOMEM;

	hstep = div_u64((u64)in_w << VSU_FRAC_BITS, out_w);
	vstep = div_u64((u64)in_h << VSU_FRAC_BITS, out_h);

	put(vsu, G2D_VSU_CTRL, G2D_VSU_CTRL_EN | G2D_VSU_CTRL_COEF_ACCESS |
	    (planar420 ? G2D_VSU_CTRL_FILTER_PLANAR : 0));
	put(vsu, G2D_VSU_OUT_SIZE, G2D_SIZE(out_w, out_h));
	put(vsu, G2D_VSU_GLB_ALPHA, 0xff);
	put(vsu, G2D_VSU_Y_SIZE, G2D_SIZE(in_w, in_h));
	/* the step registers have a reserved bit 0 */
	put(vsu, G2D_VSU_Y_HSTEP, hstep << 1);
	put(vsu, G2D_VSU_Y_VSTEP, vstep << 1);
	put_table(vsu, G2D_VSU_Y_HCOEF, sun252i_g2d_coef_lanczos[vsu_coef_set(hstep)], G2D_VSU_PHASES);

	if (planar420) {
		/* 4:2:0: chroma has half the luma resolution in both directions */
		cw = (in_w + 1) >> 1;
		ch = (in_h + 1) >> 1;
		put(vsu, G2D_VSU_C_SIZE, G2D_SIZE(cw, ch));
		put(vsu, G2D_VSU_C_HSTEP, hstep);
		put(vsu, G2D_VSU_C_VSTEP, vstep);
		put(vsu, G2D_VSU_C_HPHASE, 0xfffc0000);
		put(vsu, G2D_VSU_C_VPHASE, 0xfffc0000);
		put_table(vsu, G2D_VSU_C_HCOEF, sun252i_g2d_coef_lanczos[vsu_coef_set(hstep >> 1)],
			  G2D_VSU_PHASES);
		put_table(vsu, G2D_VSU_Y_VCOEF, sun252i_g2d_coef_lanczos[vsu_coef_set(vstep)],
			  G2D_VSU_PHASES);
	} else if (packed422) {
		/* 4:2:2: half the horizontal resolution only */
		cw = (in_w + 1) >> 1;
		put(vsu, G2D_VSU_C_SIZE, G2D_SIZE(cw, in_h));
		put(vsu, G2D_VSU_C_HSTEP, hstep);
		put(vsu, G2D_VSU_C_VSTEP, vstep << 1);
		put_table(vsu, G2D_VSU_C_HCOEF, sun252i_g2d_coef_lanczos[vsu_coef_set(hstep >> 1)],
			  G2D_VSU_PHASES);
		put_table(vsu, G2D_VSU_Y_VCOEF, sun252i_g2d_coef_linear, G2D_VSU_PHASES);
	} else {
		/* RGB: the "chroma" path carries the same samples */
		put(vsu, G2D_VSU_C_SIZE, G2D_SIZE(in_w, in_h));
		put(vsu, G2D_VSU_C_HSTEP, hstep << 1);
		put(vsu, G2D_VSU_C_VSTEP, vstep << 1);
		put_table(vsu, G2D_VSU_C_HCOEF, sun252i_g2d_coef_lanczos[vsu_coef_set(hstep)],
			  G2D_VSU_PHASES);
		put_table(vsu, G2D_VSU_Y_VCOEF, sun252i_g2d_coef_linear, G2D_VSU_PHASES);
	}
	return 0;
}

/* the scaler stays off: tell the hardware explicitly, it keeps its previous state */
static int vsu_disable(struct g2d_mixer *mix)
{
	return cl_block(mix->cl, G2D_VSU, sizeof(u32)) ? 0 : -ENOMEM;
}

static void bld_csc_yuv2rgb(struct g2d_mixer *mix, struct g2d_ctx *ctx)
{
	bool bt709 = ctx->src.ycbcr_enc == V4L2_YCBCR_ENC_709 ||
		     (ctx->src.ycbcr_enc == V4L2_YCBCR_ENC_DEFAULT &&
		      ctx->src.colorspace == V4L2_COLORSPACE_REC709);
	bool full = ctx->src.quantization == V4L2_QUANTIZATION_FULL_RANGE;
	const u32 *matrix = bt709 ? sun252i_g2d_csc_709[full] : sun252i_g2d_csc_601[full];

	put_or(mix->bld, G2D_BLD_CSC_CTRL, G2D_BLD_CSC_EN(BLD_CSC_OUTPUT));
	put(mix->bld, G2D_BLD_FILL_COLOR(0), BLD_FILL_YUV_BLACK);
	put(mix->bld, G2D_BLD_FILL_COLOR(1), BLD_FILL_YUV_BLACK);
	put_table(mix->bld, G2D_BLD_CSC(BLD_CSC_OUTPUT), matrix, G2D_CSC_WORDS);
}

static int compose_blit(struct g2d_cmdlist *cl, struct g2d_ctx *ctx,
			const struct g2d_hw_buf *src, const struct g2d_hw_buf *dst)
{
	struct g2d_mixer mix;
	int ret;

	ret = mixer_begin(&mix, cl);
	if (ret)
		return ret;

	v0_setup(&mix, src);

	if (src->fmt->yuv || src->width != dst->width || src->height != dst->height) {
		u32 mid_w, mid_h;

		v0_coarse(&mix, src->fmt, src->width, src->height, dst->width, dst->height, &mid_w, &mid_h);
		ret = vsu_enable(&mix, src->fmt, mid_w, mid_h, dst->width, dst->height);
	} else {
		ret = vsu_disable(&mix);
	}
	if (ret)
		return ret;

	/* pipe 0 is the bottom layer of the blender: it also fills the area */
	put_or(mix.bld, G2D_BLD_EN, G2D_BLD_EN_P0 | G2D_BLD_EN_P0_FILL);
	put(mix.bld, G2D_BLD_PIPE_SIZE(0), G2D_SIZE(dst->width, dst->height));
	put(mix.bld, G2D_BLD_PIPE_COOR(0), 0);
	put(mix.bld, G2D_BLD_CTRL, BLD_PIPE0_ONLY);
	put(mix.bld, G2D_BLD_OUT_SIZE, G2D_SIZE(dst->width, dst->height));
	/* YCbCr stays in its colour space through the blender, converted at the output */
	put(mix.bld, G2D_BLD_OUT_COLOR, src->fmt->yuv ? G2D_BLD_OUT_YUV : 0);
	if (src->fmt->yuv)
		bld_csc_yuv2rgb(&mix, ctx);

	put(mix.wb, G2D_WB_ATTR, FIELD_PREP(G2D_WB_ATTR_FORMAT_MASK, dst->fmt->hw));
	put(mix.wb, G2D_WB_DATA_SIZE, G2D_SIZE(dst->width, dst->height));
	put(mix.wb, G2D_WB_PITCH(0), dst->pitch[0]);
	put(mix.wb, G2D_WB_ADDR(0), dst->addr[0]);
	return 0;
}

static int compose_rotate(struct g2d_cmdlist *cl, struct g2d_ctx *ctx,
			  const struct g2d_hw_buf *src, const struct g2d_hw_buf *dst)
{
	static const u8 degree[] = { [0] = 0, [90] = 1, [180] = 2, [270] = 3 };
	__le32 *core, *rot;
	unsigned int plane;

	core = cl_block(cl, G2D_CORE_CTRL, G2D_CORE_CTRL_SIZE);
	rot = cl_block(cl, G2D_ROT, G2D_ROT_SIZE);
	if (!core || !rot)
		return -ENOMEM;

	put(core, 0, G2D_CORE_CTRL_ROTATE);
	put(rot, G2D_ROT_CTRL, FIELD_PREP(G2D_ROT_CTRL_MODE_MASK, ROT_MODE_NORMAL) |
	    FIELD_PREP(G2D_ROT_CTRL_DEGREE_MASK, degree[ctx->rotate]) |
	    (ctx->vflip ? G2D_ROT_CTRL_VFLIP : 0) | (ctx->hflip ? G2D_ROT_CTRL_HFLIP : 0));
	put(rot, G2D_ROT_IN_FORMAT, src->fmt->hw);
	put(rot, G2D_ROT_IN_SIZE, G2D_SIZE(src->width, src->height));
	put(rot, G2D_ROT_OUT_SIZE, G2D_SIZE(dst->width, dst->height));
	for (plane = 0; plane < src->fmt->planes; plane++) {
		put(rot, G2D_ROT_IN_PITCH(plane), src->pitch[plane]);
		put(rot, G2D_ROT_IN_ADDR(plane), src->addr[plane]);
		put(rot, G2D_ROT_OUT_PITCH(plane), dst->pitch[plane]);
		put(rot, G2D_ROT_OUT_ADDR(plane), dst->addr[plane]);
	}
	return 0;
}

static bool g2d_uses_rotate(const struct g2d_ctx *ctx)
{
	return ctx->rotate || ctx->hflip || ctx->vflip;
}

/* can the engine do what the formats and the controls ask for? */
static int g2d_check(const struct g2d_ctx *ctx)
{
	bool swap = ctx->rotate == 90 || ctx->rotate == 270;
	const struct v4l2_pix_format *s = &ctx->src, *d = &ctx->dst;

	if (!ctx->src_fmt || !ctx->dst_fmt)
		return -EINVAL;
	if (!g2d_uses_rotate(ctx))
		return 0;

	/* the rotation is a move: same format, no scaling */
	if (ctx->src_fmt != ctx->dst_fmt ||
	    d->width != (swap ? s->height : s->width) ||
	    d->height != (swap ? s->width : s->height))
		return -EINVAL;
	return 0;
}

static void g2d_hw_buf_init(struct g2d_hw_buf *buf, const struct g2d_fmt *fmt,
			    const struct v4l2_pix_format *pix, dma_addr_t base)
{
	unsigned int plane;
	u32 offset = 0;

	buf->fmt = fmt;
	buf->width = pix->width;
	buf->height = pix->height;
	memset(buf->addr, 0, sizeof(buf->addr));
	memset(buf->pitch, 0, sizeof(buf->pitch));
	for (plane = 0; plane < fmt->planes; plane++) {
		buf->pitch[plane] = g2d_plane_pitch(fmt, plane, pix->bytesperline);
		buf->addr[plane] = base + offset;
		offset += g2d_plane_size(fmt, plane, pix->bytesperline, pix->height);
	}
}

/* ---- hardware ------------------------------------------------------------ */

static void g2d_write(struct g2d_dev *dev, u32 reg, u32 value)
{
	writel(value, dev->regs + reg);
}

static void g2d_update(struct g2d_dev *dev, u32 reg, u32 mask, u32 value)
{
	g2d_write(dev, reg, (readl(dev->regs + reg) & ~mask) | value);
}

static void g2d_pulse_reset(struct g2d_dev *dev, u32 bits)
{
	g2d_update(dev, G2D_RESET, bits, 0);
	g2d_update(dev, G2D_RESET, bits, bits);
}

static void g2d_hw_open(struct g2d_dev *dev)
{
	g2d_update(dev, G2D_CLK_GATE, G2D_CLK_GATE_CORE, G2D_CLK_GATE_CORE);
	g2d_update(dev, G2D_MBUS_GATE, G2D_MBUS_GATE_CLK | G2D_MBUS_GATE_RESET,
		   G2D_MBUS_GATE_CLK | G2D_MBUS_GATE_RESET);
	g2d_update(dev, G2D_RESET, G2D_RESET_CORE, G2D_RESET_CORE);
	g2d_update(dev, G2D_CLK_GATE, G2D_CLK_GATE_THREAD(G2D_THREAD_ID), G2D_CLK_GATE_THREAD(G2D_THREAD_ID));
	g2d_update(dev, G2D_RESET, G2D_RESET_THREAD(G2D_THREAD_ID), G2D_RESET_THREAD(G2D_THREAD_ID));
	g2d_update(dev, G2D_CACHE_CTRL, G2D_CACHE_CTRL_EN, G2D_CACHE_CTRL_EN);
	g2d_update(dev, G2D_CACHE_SIZE0, G2D_CACHE_SIZE_MASK, G2D_CACHE_SIZE_VALUE);
	g2d_update(dev, G2D_CACHE_SIZE1, G2D_CACHE_SIZE_MASK, G2D_CACHE_SIZE_VALUE);
	/* keep the G2D from starving the display engine and the CPU of memory bandwidth */
	g2d_update(dev, G2D_DDR_LIMIT, G2D_DDR_LIMIT_EN | G2D_DDR_LIMIT_MASK,
		   G2D_DDR_LIMIT_EN | G2D_DDR_LIMIT_VALUE);
}

static void g2d_hw_recover(struct g2d_dev *dev)
{
	g2d_pulse_reset(dev, G2D_RESET_CORE);
	g2d_pulse_reset(dev, G2D_RESET_THREAD(G2D_THREAD_ID));
}

static void g2d_hw_start(struct g2d_dev *dev, size_t head_len)
{
	const u32 thread = G2D_THREAD(G2D_THREAD_ID);

	g2d_update(dev, G2D_MODE, G2D_MODE_MASTER, G2D_MODE_MASTER);
	g2d_write(dev, thread + G2D_THREAD_IRQ_EN, G2D_THREAD_IRQ_TASK_END | G2D_THREAD_IRQ_TIMEOUT);
	g2d_write(dev, thread + G2D_THREAD_HEAD_LOW, lower_32_bits(dev->cl_dma));
	g2d_write(dev, thread + G2D_THREAD_HEAD_HIGH_LEN, FIELD_PREP(G2D_THREAD_HEAD_LEN_MASK, head_len));
	g2d_write(dev, thread + G2D_THREAD_ATTR,
		  FIELD_PREP(G2D_THREAD_ATTR_CMD_NUM_MASK, 0) | G2D_THREAD_ATTR_END_IRQ);
	g2d_write(dev, thread + G2D_THREAD_UPDATE, 1);
}

static void g2d_job_done(struct g2d_dev *dev, struct g2d_ctx *ctx, enum vb2_buffer_state state)
{
	struct vb2_v4l2_buffer *src, *dst;

	src = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

	v4l2_m2m_buf_copy_metadata(src, dst);
	dst->sequence = ctx->sequence_cap++;
	src->sequence = ctx->sequence_out++;

	v4l2_m2m_buf_done(src, state);
	v4l2_m2m_buf_done(dst, state);
	v4l2_m2m_job_finish(dev->m2m_dev, ctx->fh.m2m_ctx);
}

static irqreturn_t g2d_irq(int irq, void *data)
{
	struct g2d_dev *dev = data;
	const u32 status_reg = G2D_THREAD(G2D_THREAD_ID) + G2D_THREAD_IRQ_STATUS;
	struct g2d_ctx *ctx;
	unsigned long flags;
	u32 status;

	status = readl(dev->regs + status_reg);
	g2d_write(dev, status_reg, status); /* write 1 to clear */
	if (!(status & (G2D_THREAD_IRQ_TASK_END | G2D_THREAD_IRQ_TIMEOUT)))
		return IRQ_NONE;

	g2d_pulse_reset(dev, G2D_RESET_THREAD(G2D_THREAD_ID));

	spin_lock_irqsave(&dev->lock, flags);
	ctx = dev->running;
	dev->running = NULL;
	spin_unlock_irqrestore(&dev->lock, flags);
	if (!ctx)
		return IRQ_HANDLED;

	cancel_delayed_work(&dev->watchdog);
	if (status & G2D_THREAD_IRQ_TIMEOUT)
		g2d_hw_recover(dev);
	g2d_job_done(dev, ctx, (status & G2D_THREAD_IRQ_TIMEOUT) ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
	return IRQ_HANDLED;
}

static void g2d_watchdog(struct work_struct *work)
{
	struct g2d_dev *dev = container_of(work, struct g2d_dev, watchdog.work);
	struct g2d_ctx *ctx;
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	ctx = dev->running;
	dev->running = NULL;
	spin_unlock_irqrestore(&dev->lock, flags);
	if (!ctx)
		return;

	dev_err(dev->dev, "the engine did not finish the frame\n");
	g2d_hw_recover(dev);
	g2d_job_done(dev, ctx, VB2_BUF_STATE_ERROR);
}

/* ---- mem2mem ------------------------------------------------------------- */

static void device_run(void *priv)
{
	struct g2d_ctx *ctx = priv;
	struct g2d_dev *dev = ctx->dev;
	struct vb2_v4l2_buffer *src, *dst;
	struct g2d_hw_buf src_buf, dst_buf;
	struct g2d_cmdlist cl;
	unsigned long flags;
	int ret;

	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);

	g2d_hw_buf_init(&src_buf, ctx->src_fmt, &ctx->src, vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0));
	g2d_hw_buf_init(&dst_buf, ctx->dst_fmt, &ctx->dst, vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0));

	cl_init(&cl, dev->cl_cpu, dev->cl_dma);
	if (g2d_uses_rotate(ctx))
		ret = compose_rotate(&cl, ctx, &src_buf, &dst_buf);
	else
		ret = compose_blit(&cl, ctx, &src_buf, &dst_buf);
	if (ret) {
		g2d_job_done(dev, ctx, VB2_BUF_STATE_ERROR);
		return;
	}

	spin_lock_irqsave(&dev->lock, flags);
	dev->running = ctx;
	spin_unlock_irqrestore(&dev->lock, flags);

	schedule_delayed_work(&dev->watchdog, msecs_to_jiffies(G2D_TIMEOUT_MS));
	/* the list sits in coherent memory: the hardware sees it as written */
	wmb();
	g2d_hw_start(dev, cl_finish(&cl));
}

static const struct v4l2_m2m_ops g2d_m2m_ops = {
	.device_run = device_run,
};

/* ---- vb2 ----------------------------------------------------------------- */

static struct v4l2_pix_format *g2d_pix(struct g2d_ctx *ctx, enum v4l2_buf_type type)
{
	return V4L2_TYPE_IS_OUTPUT(type) ? &ctx->src : &ctx->dst;
}

static int g2d_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers, unsigned int *nplanes,
			   unsigned int sizes[], struct device *alloc_devs[])
{
	struct g2d_ctx *ctx = vb2_get_drv_priv(vq);
	const struct v4l2_pix_format *pix = g2d_pix(ctx, vq->type);

	if (*nplanes)
		return sizes[0] < pix->sizeimage ? -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = pix->sizeimage;
	return 0;
}

static int g2d_buf_prepare(struct vb2_buffer *vb)
{
	struct g2d_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	const struct v4l2_pix_format *pix = g2d_pix(ctx, vb->vb2_queue->type);

	if (vb2_plane_size(vb, 0) < pix->sizeimage)
		return -EINVAL;
	vb2_set_plane_payload(vb, 0, pix->sizeimage);
	return 0;
}

static void g2d_buf_queue(struct vb2_buffer *vb)
{
	struct g2d_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, to_vb2_v4l2_buffer(vb));
}

static void g2d_return_buffers(struct g2d_ctx *ctx, struct vb2_queue *vq)
{
	struct vb2_v4l2_buffer *vbuf;

	for (;;) {
		if (V4L2_TYPE_IS_OUTPUT(vq->type))
			vbuf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		else
			vbuf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (!vbuf)
			break;
		v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_QUEUED);
	}
}

static int g2d_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct g2d_ctx *ctx = vb2_get_drv_priv(vq);

	if (V4L2_TYPE_IS_OUTPUT(vq->type))
		ctx->sequence_out = 0;
	else
		ctx->sequence_cap = 0;

	if (g2d_check(ctx)) {
		g2d_return_buffers(ctx, vq);
		return -EINVAL;
	}
	return 0;
}

static void g2d_stop_streaming(struct vb2_queue *vq)
{
	struct g2d_ctx *ctx = vb2_get_drv_priv(vq);

	g2d_return_buffers(ctx, vq);
}

static const struct vb2_ops g2d_qops = {
	.queue_setup = g2d_queue_setup,
	.buf_prepare = g2d_buf_prepare,
	.buf_queue = g2d_buf_queue,
	.start_streaming = g2d_start_streaming,
	.stop_streaming = g2d_stop_streaming,
};

static int g2d_queue_init(void *priv, struct vb2_queue *src_vq, struct vb2_queue *dst_vq)
{
	struct g2d_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->ops = &g2d_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->dev->mutex;
	src_vq->dev = ctx->dev->dev;
	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->ops = &g2d_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->dev->mutex;
	dst_vq->dev = ctx->dev->dev;
	return vb2_queue_init(dst_vq);
}

/* ---- ioctls --------------------------------------------------------------- */

static int g2d_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	strscpy(cap->driver, G2D_NAME, sizeof(cap->driver));
	strscpy(cap->card, "Allwinner F101 G2D", sizeof(cap->card));
	return 0;
}

static int g2d_enum_fmt(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	bool capture = f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE;
	unsigned int i, index = 0;

	for (i = 0; i < ARRAY_SIZE(g2d_formats); i++) {
		if (capture && !g2d_formats[i].dst)
			continue;
		if (index++ == f->index) {
			f->pixelformat = g2d_formats[i].fourcc;
			return 0;
		}
	}
	return -EINVAL;
}

static int g2d_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct g2d_ctx *ctx = file2ctx(file);

	f->fmt.pix = *g2d_pix(ctx, f->type);
	return 0;
}

static int g2d_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct g2d_ctx *ctx = file2ctx(file);
	struct v4l2_pix_format *pix = &f->fmt.pix;
	bool capture = f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE;
	const struct g2d_fmt *fmt = g2d_find_fmt(pix->pixelformat, capture);

	if (!fmt)
		fmt = g2d_find_fmt(capture ? V4L2_PIX_FMT_XBGR32 : V4L2_PIX_FMT_NV12, capture);

	g2d_fill_pix(pix, fmt);
	if (capture) {
		pix->colorspace = ctx->src.colorspace;
		pix->xfer_func = ctx->src.xfer_func;
		pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
		pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	} else {
		if (!pix->colorspace)
			pix->colorspace = V4L2_COLORSPACE_REC709;
		if (!fmt->yuv) {
			pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
			pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
		}
	}
	return 0;
}

static int g2d_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct g2d_ctx *ctx = file2ctx(file);
	struct vb2_queue *vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	bool capture = f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE;
	int ret;

	if (vb2_is_busy(vq))
		return -EBUSY;

	ret = g2d_try_fmt(file, priv, f);
	if (ret)
		return ret;

	if (capture) {
		ctx->dst = f->fmt.pix;
		ctx->dst_fmt = g2d_find_fmt(ctx->dst.pixelformat, true);
	} else {
		ctx->src = f->fmt.pix;
		ctx->src_fmt = g2d_find_fmt(ctx->src.pixelformat, false);
		g2d_fill_colorspace(ctx);
	}
	return 0;
}

static const struct v4l2_ioctl_ops g2d_ioctl_ops = {
	.vidioc_querycap = g2d_querycap,

	.vidioc_enum_fmt_vid_cap = g2d_enum_fmt,
	.vidioc_g_fmt_vid_cap = g2d_g_fmt,
	.vidioc_try_fmt_vid_cap = g2d_try_fmt,
	.vidioc_s_fmt_vid_cap = g2d_s_fmt,

	.vidioc_enum_fmt_vid_out = g2d_enum_fmt,
	.vidioc_g_fmt_vid_out = g2d_g_fmt,
	.vidioc_try_fmt_vid_out = g2d_try_fmt,
	.vidioc_s_fmt_vid_out = g2d_s_fmt,

	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,

	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,

	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

/* ---- controls ------------------------------------------------------------- */

static int g2d_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct g2d_ctx *ctx = container_of(ctrl->handler, struct g2d_ctx, ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_ROTATE:
		ctx->rotate = ctrl->val;
		break;
	case V4L2_CID_HFLIP:
		ctx->hflip = ctrl->val;
		break;
	case V4L2_CID_VFLIP:
		ctx->vflip = ctrl->val;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static const struct v4l2_ctrl_ops g2d_ctrl_ops = {
	.s_ctrl = g2d_s_ctrl,
};

/* ---- file operations ------------------------------------------------------ */

static int g2d_open(struct file *file)
{
	struct g2d_dev *dev = video_drvdata(file);
	struct g2d_ctx *ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	if (mutex_lock_interruptible(&dev->mutex)) {
		kfree(ctx);
		return -ERESTARTSYS;
	}

	ctx->dev = dev;
	v4l2_fh_init(&ctx->fh, video_devdata(file));

	v4l2_ctrl_handler_init(&ctx->ctrl_handler, 3);
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &g2d_ctrl_ops, V4L2_CID_ROTATE, 0, 270, 90, 0);
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &g2d_ctrl_ops, V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &g2d_ctrl_ops, V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (ctx->ctrl_handler.error) {
		ret = ctx->ctrl_handler.error;
		goto err_fh;
	}
	ctx->fh.ctrl_handler = &ctx->ctrl_handler;

	ctx->src.width = 640;
	ctx->src.height = 480;
	ctx->src.colorspace = V4L2_COLORSPACE_REC709;
	ctx->src_fmt = g2d_find_fmt(V4L2_PIX_FMT_NV12, false);
	g2d_fill_pix(&ctx->src, ctx->src_fmt);
	ctx->dst = ctx->src;
	ctx->dst_fmt = g2d_find_fmt(V4L2_PIX_FMT_XBGR32, true);
	g2d_fill_pix(&ctx->dst, ctx->dst_fmt);
	g2d_fill_colorspace(ctx);

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(dev->m2m_dev, ctx, g2d_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto err_fh;
	}

	file->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh, file);
	mutex_unlock(&dev->mutex);
	return 0;

err_fh:
	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	v4l2_fh_exit(&ctx->fh);
	mutex_unlock(&dev->mutex);
	kfree(ctx);
	return ret;
}

static int g2d_release(struct file *file)
{
	struct g2d_dev *dev = video_drvdata(file);
	struct g2d_ctx *ctx = file2ctx(file);

	mutex_lock(&dev->mutex);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	mutex_unlock(&dev->mutex);

	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return 0;
}

static const struct v4l2_file_operations g2d_fops = {
	.owner = THIS_MODULE,
	.open = g2d_open,
	.release = g2d_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

static const struct video_device g2d_videodev = {
	.name = G2D_NAME,
	.vfl_dir = VFL_DIR_M2M,
	.fops = &g2d_fops,
	.ioctl_ops = &g2d_ioctl_ops,
	.minor = -1,
	.release = video_device_release_empty,
	.device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING,
};

/* ---- platform driver ------------------------------------------------------- */

static int g2d_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct g2d_dev *g2d;
	int irq, ret;

	g2d = devm_kzalloc(dev, sizeof(*g2d), GFP_KERNEL);
	if (!g2d)
		return -ENOMEM;
	g2d->dev = dev;
	platform_set_drvdata(pdev, g2d);
	mutex_init(&g2d->mutex);
	spin_lock_init(&g2d->lock);
	INIT_DELAYED_WORK(&g2d->watchdog, g2d_watchdog);

	g2d->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(g2d->regs))
		return PTR_ERR(g2d->regs);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	g2d->clk_mod = devm_clk_get(dev, "mod");
	if (IS_ERR(g2d->clk_mod))
		return dev_err_probe(dev, PTR_ERR(g2d->clk_mod), "module clock\n");
	g2d->clk_bus = devm_clk_get(dev, "bus");
	if (IS_ERR(g2d->clk_bus))
		return dev_err_probe(dev, PTR_ERR(g2d->clk_bus), "bus clock\n");
	g2d->clk_mbus = devm_clk_get(dev, "mbus");
	if (IS_ERR(g2d->clk_mbus))
		return dev_err_probe(dev, PTR_ERR(g2d->clk_mbus), "memory bus clock\n");
	g2d->rst = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(g2d->rst))
		return dev_err_probe(dev, PTR_ERR(g2d->rst), "reset\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	g2d->cl_cpu = dmam_alloc_coherent(dev, G2D_CL_BYTES, &g2d->cl_dma, GFP_KERNEL);
	if (!g2d->cl_cpu)
		return -ENOMEM;

	ret = clk_set_rate(g2d->clk_mod, G2D_MODULE_RATE);
	if (ret)
		return dev_err_probe(dev, ret, "module clock rate\n");
	ret = clk_prepare_enable(g2d->clk_bus);
	if (ret)
		return ret;
	ret = clk_prepare_enable(g2d->clk_mbus);
	if (ret)
		goto err_bus;
	ret = reset_control_reset(g2d->rst);
	if (ret)
		goto err_mbus;
	ret = clk_prepare_enable(g2d->clk_mod);
	if (ret)
		goto err_rst;
	g2d_hw_open(g2d);

	ret = devm_request_irq(dev, irq, g2d_irq, 0, G2D_NAME, g2d);
	if (ret)
		goto err_mod;

	ret = v4l2_device_register(dev, &g2d->v4l2_dev);
	if (ret)
		goto err_mod;

	g2d->vfd = g2d_videodev;
	g2d->vfd.lock = &g2d->mutex;
	g2d->vfd.v4l2_dev = &g2d->v4l2_dev;
	video_set_drvdata(&g2d->vfd, g2d);

	g2d->m2m_dev = v4l2_m2m_init(&g2d_m2m_ops);
	if (IS_ERR(g2d->m2m_dev)) {
		ret = PTR_ERR(g2d->m2m_dev);
		goto err_v4l2;
	}

	ret = video_register_device(&g2d->vfd, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_m2m;

	dev_info(dev, "registered as /dev/video%d\n", g2d->vfd.num);
	return 0;

err_m2m:
	v4l2_m2m_release(g2d->m2m_dev);
err_v4l2:
	v4l2_device_unregister(&g2d->v4l2_dev);
err_mod:
	clk_disable_unprepare(g2d->clk_mod);
err_rst:
	reset_control_assert(g2d->rst);
err_mbus:
	clk_disable_unprepare(g2d->clk_mbus);
err_bus:
	clk_disable_unprepare(g2d->clk_bus);
	return ret;
}

static void g2d_remove(struct platform_device *pdev)
{
	struct g2d_dev *g2d = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&g2d->watchdog);
	video_unregister_device(&g2d->vfd);
	v4l2_m2m_release(g2d->m2m_dev);
	v4l2_device_unregister(&g2d->v4l2_dev);
	clk_disable_unprepare(g2d->clk_mod);
	reset_control_assert(g2d->rst);
	clk_disable_unprepare(g2d->clk_mbus);
	clk_disable_unprepare(g2d->clk_bus);
}

static const struct of_device_id g2d_of_match[] = {
	{ .compatible = "allwinner,sun252i-f101-g2d" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, g2d_of_match);

static struct platform_driver g2d_driver = {
	.probe = g2d_probe,
	.remove = g2d_remove,
	.driver = {
		.name = G2D_NAME,
		.of_match_table = g2d_of_match,
	},
};
module_platform_driver(g2d_driver);

MODULE_DESCRIPTION("Allwinner F101 G2D memory to memory driver");
MODULE_AUTHOR("Yuzuki Tsuru <gloomyghost@gloomyghost.com>");
MODULE_LICENSE("GPL");
