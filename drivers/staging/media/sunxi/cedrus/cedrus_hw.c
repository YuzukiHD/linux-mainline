// SPDX-License-Identifier: GPL-2.0
/*
 * Cedrus VPU driver
 *
 * Copyright (C) 2016 Florent Revest <florent.revest@free-electrons.com>
 * Copyright (C) 2018 Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 * Copyright (C) 2018 Bootlin
 *
 * Based on the vim2m driver, that is:
 *
 * Copyright (c) 2009-2010 Samsung Electronics Co., Ltd.
 * Pawel Osciak, <pawel@osciak.com>
 * Marek Szyprowski, <m.szyprowski@samsung.com>
 */

#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/soc/sunxi/sunxi_sram.h>

#include <media/videobuf2-core.h>
#include <media/v4l2-mem2mem.h>

#include "cedrus.h"
#include "cedrus_hw.h"
#include "cedrus_regs.h"

int cedrus_engine_enable(struct cedrus_ctx *ctx)
{
	bool inverted = cedrus_is_capable(ctx, CEDRUS_CAPABILITY_MODE_INVERTED);
	u32 reg = 0;

	/*
	 * FIXME: This is only valid on 32-bits DDR's, we should test
	 * it on the A13/A33.
	 */
	if (!inverted) {
		reg |= VE_MODE_REC_WR_MODE_2MB;
		reg |= VE_MODE_DDR_MODE_BW_128;
	}

	switch (ctx->src_fmt.pixelformat) {
	case V4L2_PIX_FMT_MPEG2_SLICE:
	case V4L2_PIX_FMT_JPEG:
		reg |= VE_MODE_DEC_MPEG;
		break;

	/* H.264 and VP8 both use the same decoding mode bit. */
	case V4L2_PIX_FMT_H264_SLICE:
	case V4L2_PIX_FMT_VP8_FRAME:
		reg |= VE_MODE_DEC_H264;
		break;

	case V4L2_PIX_FMT_HEVC_SLICE:
		reg |= VE_MODE_DEC_H265;
		break;

	default:
		return -EINVAL;
	}

	if (inverted)
		reg ^= VE_MODE_INVERT;

	if (ctx->src_fmt.width == 4096)
		reg |= VE_MODE_PIC_WIDTH_IS_4096;
	if (ctx->src_fmt.width > 2048)
		reg |= VE_MODE_PIC_WIDTH_MORE_2048;

	cedrus_write(ctx->dev, VE_MODE, reg);

	return 0;
}

void cedrus_engine_disable(struct cedrus_dev *dev)
{
	cedrus_write(dev, VE_MODE, VE_MODE_DISABLED);
}

void cedrus_dst_format_set(struct cedrus_dev *dev,
			   struct v4l2_pix_format *fmt)
{
	unsigned int width = fmt->width;
	unsigned int height = fmt->height;
	u32 chroma_size;
	u32 reg;

	switch (fmt->pixelformat) {
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YVU420:
		chroma_size = ALIGN(width, 16) * ALIGN(height, 16) / 2;

		switch (fmt->pixelformat) {
		case V4L2_PIX_FMT_NV12:
			reg = VE_PRIMARY_OUT_FMT_NV12;
			break;
		case V4L2_PIX_FMT_NV21:
			reg = VE_PRIMARY_OUT_FMT_NV21;
			break;
		case V4L2_PIX_FMT_YUV420:
			reg = VE_PRIMARY_OUT_FMT_YU12;
			break;
		case V4L2_PIX_FMT_YVU420:
		default:
			reg = VE_PRIMARY_OUT_FMT_YV12;
			break;
		}
		cedrus_write(dev, VE_PRIMARY_OUT_FMT, reg);

		reg = chroma_size / 2;
		cedrus_write(dev, VE_PRIMARY_CHROMA_BUF_LEN, reg);

		reg = VE_PRIMARY_FB_LINE_STRIDE_LUMA(ALIGN(width, 16)) |
		      VE_PRIMARY_FB_LINE_STRIDE_CHROMA(ALIGN(width, 16) / 2);
		cedrus_write(dev, VE_PRIMARY_FB_LINE_STRIDE, reg);

		break;
	case V4L2_PIX_FMT_NV12_32L32:
	default:
		/* the F101 keeps the tiled output at the reset value of the registers */
		if (dev->capabilities & CEDRUS_CAPABILITY_MODE_INVERTED)
			break;
		reg = VE_PRIMARY_OUT_FMT_TILED_32_NV12;
		cedrus_write(dev, VE_PRIMARY_OUT_FMT, reg);

		reg = VE_SECONDARY_OUT_FMT_TILED_32_NV12;
		cedrus_write(dev, VE_CHROMA_BUF_LEN, reg);

		break;
	}
}

/* Conversion block: tiled picture (chroma at width * height) to linear NV12 */
#define CV_SIZE		0xa00
#define CV_STRIDE	0xa04
#define CV_ENABLE	0xa08
#define CV_MODE		0xa0c
#define CV_STATUS	0xa10
#define CV_LINE		0xa14
#define CV_CONFIG	0xa2c
#define CV_OUT_Y	0xa70
#define CV_OUT_C	0xa74
#define CV_IN_Y		0xa78
#define CV_IN_C		0xa7c
#define CV_TOP_MODE	2

void cedrus_convert_start(struct cedrus_dev *dev, u32 width, u32 height,
			  dma_addr_t in, dma_addr_t out)
{
	u32 mbw = width / 16;
	u32 mbh = height / 16;

	cedrus_write(dev, 0x04, 1);
	cedrus_write(dev, 0x04, 0);
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
	cedrus_write(dev, CV_OUT_Y, out);
	cedrus_write(dev, CV_OUT_C, out + width * height);
	cedrus_write(dev, CV_IN_Y, in);
	cedrus_write(dev, CV_IN_C, in + width * height);
	cedrus_write(dev, CV_MODE, 1);
}

bool cedrus_convert_done(struct cedrus_dev *dev)
{
	return (cedrus_read(dev, CV_ENABLE) & 1) &&
	       (cedrus_read(dev, CV_STATUS) & 1);
}

void cedrus_convert_clear(struct cedrus_dev *dev)
{
	cedrus_write(dev, CV_ENABLE, cedrus_read(dev, CV_ENABLE) & ~1);
	cedrus_write(dev, CV_STATUS, 1);
}

static irqreturn_t cedrus_irq(int irq, void *data)
{
	struct cedrus_dev *dev = data;
	struct cedrus_ctx *ctx;
	enum vb2_buffer_state state;
	enum cedrus_irq_status status;

	/*
	 * If cancel_delayed_work returns false it means watchdog already
	 * executed and finished the job.
	 */
	if (!cancel_delayed_work(&dev->watchdog_work))
		return IRQ_HANDLED;

	ctx = v4l2_m2m_get_curr_priv(dev->m2m_dev);
	if (!ctx) {
		v4l2_err(&dev->v4l2_dev,
			 "Instance released before the end of transaction\n");
		return IRQ_NONE;
	}

	status = ctx->current_codec->irq_status(ctx);
	if (status == CEDRUS_IRQ_NONE)
		return IRQ_NONE;

	if (status == CEDRUS_IRQ_CONTINUE) {
		schedule_delayed_work(&dev->watchdog_work,
				      msecs_to_jiffies(2000));
		return IRQ_HANDLED;
	}

	ctx->current_codec->irq_disable(ctx);
	ctx->current_codec->irq_clear(ctx);

	if (status == CEDRUS_IRQ_ERROR)
		state = VB2_BUF_STATE_ERROR;
	else
		state = VB2_BUF_STATE_DONE;

	v4l2_m2m_buf_done_and_job_finish(ctx->dev->m2m_dev, ctx->fh.m2m_ctx,
					 state);

	return IRQ_HANDLED;
}

void cedrus_watchdog(struct work_struct *work)
{
	struct cedrus_dev *dev;
	struct cedrus_ctx *ctx;

	dev = container_of(to_delayed_work(work),
			   struct cedrus_dev, watchdog_work);

	ctx = v4l2_m2m_get_curr_priv(dev->m2m_dev);
	if (!ctx)
		return;

	v4l2_err(&dev->v4l2_dev, "frame processing timed out!\n");
	reset_control_reset(dev->rstc);
	v4l2_m2m_buf_done_and_job_finish(ctx->dev->m2m_dev, ctx->fh.m2m_ctx,
					 VB2_BUF_STATE_ERROR);
}

int cedrus_hw_suspend(struct device *device)
{
	struct cedrus_dev *dev = dev_get_drvdata(device);

	clk_disable_unprepare(dev->ram_clk);
	clk_disable_unprepare(dev->mod_clk);
	clk_disable_unprepare(dev->ahb_clk);

	reset_control_assert(dev->rstc);

	return 0;
}

int cedrus_hw_resume(struct device *device)
{
	struct cedrus_dev *dev = dev_get_drvdata(device);
	int ret;

	ret = reset_control_reset(dev->rstc);
	if (ret) {
		dev_err(dev->dev, "Failed to apply reset\n");

		return ret;
	}

	ret = clk_prepare_enable(dev->ahb_clk);
	if (ret) {
		dev_err(dev->dev, "Failed to enable AHB clock\n");

		goto err_rst;
	}

	ret = clk_prepare_enable(dev->mod_clk);
	if (ret) {
		dev_err(dev->dev, "Failed to enable MOD clock\n");

		goto err_ahb_clk;
	}

	ret = clk_prepare_enable(dev->ram_clk);
	if (ret) {
		dev_err(dev->dev, "Failed to enable RAM clock\n");

		goto err_mod_clk;
	}

	return 0;

err_mod_clk:
	clk_disable_unprepare(dev->mod_clk);
err_ahb_clk:
	clk_disable_unprepare(dev->ahb_clk);
err_rst:
	reset_control_assert(dev->rstc);

	return ret;
}

int cedrus_hw_probe(struct cedrus_dev *dev)
{
	const struct cedrus_variant *variant;
	int irq_dec;
	int ret;

	variant = of_device_get_match_data(dev->dev);
	if (!variant)
		return -EINVAL;

	dev->capabilities = variant->capabilities;

	irq_dec = platform_get_irq(dev->pdev, 0);
	if (irq_dec <= 0)
		return irq_dec;
	ret = devm_request_irq(dev->dev, irq_dec, cedrus_irq,
			       0, dev_name(dev->dev), dev);
	if (ret) {
		dev_err(dev->dev, "Failed to request IRQ\n");

		return ret;
	}

	ret = of_reserved_mem_device_init(dev->dev);
	if (ret && ret != -ENODEV) {
		dev_err(dev->dev, "Failed to reserve memory\n");

		return ret;
	}

	ret = sunxi_sram_claim(dev->dev);
	if (ret) {
		dev_err(dev->dev, "Failed to claim SRAM\n");

		goto err_mem;
	}

	dev->ahb_clk = devm_clk_get(dev->dev, "ahb");
	if (IS_ERR(dev->ahb_clk)) {
		dev_err(dev->dev, "Failed to get AHB clock\n");

		ret = PTR_ERR(dev->ahb_clk);
		goto err_sram;
	}

	dev->mod_clk = devm_clk_get(dev->dev, "mod");
	if (IS_ERR(dev->mod_clk)) {
		dev_err(dev->dev, "Failed to get MOD clock\n");

		ret = PTR_ERR(dev->mod_clk);
		goto err_sram;
	}

	dev->ram_clk = devm_clk_get(dev->dev, "ram");
	if (IS_ERR(dev->ram_clk)) {
		dev_err(dev->dev, "Failed to get RAM clock\n");

		ret = PTR_ERR(dev->ram_clk);
		goto err_sram;
	}

	dev->rstc = devm_reset_control_get(dev->dev, NULL);
	if (IS_ERR(dev->rstc)) {
		dev_err(dev->dev, "Failed to get reset control\n");

		ret = PTR_ERR(dev->rstc);
		goto err_sram;
	}

	dev->base = devm_platform_ioremap_resource(dev->pdev, 0);
	if (IS_ERR(dev->base)) {
		dev_err(dev->dev, "Failed to map registers\n");

		ret = PTR_ERR(dev->base);
		goto err_sram;
	}

	ret = clk_set_rate(dev->mod_clk, variant->mod_rate);
	if (ret) {
		dev_err(dev->dev, "Failed to set clock rate\n");

		goto err_sram;
	}

	pm_runtime_enable(dev->dev);
	if (!pm_runtime_enabled(dev->dev)) {
		ret = cedrus_hw_resume(dev->dev);
		if (ret)
			goto err_pm;
	}

	return 0;

err_pm:
	pm_runtime_disable(dev->dev);
err_sram:
	sunxi_sram_release(dev->dev);
err_mem:
	of_reserved_mem_device_release(dev->dev);

	return ret;
}

void cedrus_hw_remove(struct cedrus_dev *dev)
{
	pm_runtime_disable(dev->dev);
	if (!pm_runtime_status_suspended(dev->dev))
		cedrus_hw_suspend(dev->dev);

	sunxi_sram_release(dev->dev);

	of_reserved_mem_device_release(dev->dev);
}
