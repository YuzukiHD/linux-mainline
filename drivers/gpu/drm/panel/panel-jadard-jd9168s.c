// SPDX-License-Identifier: GPL-2.0
/*
 * Jadard JD9168S based 1024x768 MIPI DSI panel, 4 lanes.
 *
 * The init sequence is the one of the panel of the Yuzukineko carrier board.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

#include <video/mipi_display.h>

struct jd9168s {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset;
};

#define JD9168S_CMD(...)	{ __VA_ARGS__ }

static const u8 jd9168s_init[][40] = {
	JD9168S_CMD(0xdf, 0x91, 0x68, 0xf9),
	JD9168S_CMD(0xde, 0x00),
	JD9168S_CMD(0xb2, 0x00, 0x7e),
	JD9168S_CMD(0xb3, 0x00, 0x7e),
	JD9168S_CMD(0xc1, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00),
	JD9168S_CMD(0xbb, 0x02, 0x24, 0x07, 0x61, 0x19, 0x44, 0x44),
	JD9168S_CMD(0xbe, 0x1a, 0xf2),
	JD9168S_CMD(0xc3, 0x10, 0x17, 0x5a, 0x17, 0x5a, 0x05, 0x05, 0x05, 0x05, 0x15, 0x15, 0x31, 0x05, 0xdf),
	JD9168S_CMD(0xc4, 0x11, 0x80, 0x00, 0xdf, 0x09, 0x06, 0x14),
	JD9168S_CMD(0xce, 0x00, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x0f, 0x03),
	JD9168S_CMD(0xcf, 0x00, 0x01, 0x40, 0x01, 0xca, 0x01, 0xca, 0x01, 0xca),
	JD9168S_CMD(0xd0, 0x00, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x03, 0x01, 0x05, 0x07, 0x09, 0x0b, 0x1e, 0x15, 0x1f, 0x1f, 0x15, 0x1f),
	JD9168S_CMD(0xd1, 0x00, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x02, 0x00, 0x04, 0x06, 0x08, 0x0a, 0x1e, 0x15, 0x1f, 0x1f, 0x15, 0x1f),
	JD9168S_CMD(0xd2, 0x00, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x00, 0x02, 0x0a, 0x08, 0x06, 0x04, 0x1f, 0x15, 0x1f, 0x1f, 0x15, 0x1e),
	JD9168S_CMD(0xd3, 0x00, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x01, 0x03, 0x0b, 0x09, 0x07, 0x05, 0x1f, 0x15, 0x1f, 0x1f, 0x15, 0x1e),
	JD9168S_CMD(0xd4, 0x30, 0x00, 0x00, 0x04, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x00, 0x11, 0x00, 0x01, 0xc0, 0x04, 0x01, 0x01, 0x11, 0x80, 0x01, 0xc0, 0x05, 0x01, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x06, 0x18, 0x02, 0xe3),
	JD9168S_CMD(0xd5, 0x68, 0x73, 0x00, 0x08, 0x08, 0x00, 0x03, 0x00),
	JD9168S_CMD(0xb7, 0x00, 0xd8, 0x00, 0x00, 0xd8, 0x00),
	JD9168S_CMD(0xc8, 0x7f, 0x69, 0x5a, 0x4e, 0x4a, 0x3b, 0x40, 0x2a, 0x44, 0x43, 0x44, 0x63, 0x51, 0x59, 0x4c, 0x48, 0x3a, 0x28, 0x0f, 0x7f, 0x69, 0x5a, 0x4e, 0x4a, 0x3b, 0x40, 0x2a, 0x44, 0x43, 0x44, 0x63, 0x51, 0x59, 0x4c, 0x48, 0x3a, 0x28, 0x0f),
	JD9168S_CMD(0xde, 0x02),
	JD9168S_CMD(0xbb, 0x00, 0x5b, 0x5c, 0x41),
	JD9168S_CMD(0xb5, 0x00, 0x5a, 0x0a),
	JD9168S_CMD(0xc6, 0x22),
	JD9168S_CMD(0xd7, 0x12),
	JD9168S_CMD(0xe7, 0x00, 0x00),
	JD9168S_CMD(0xde, 0x04),
	JD9168S_CMD(0xcc, 0x02),
	JD9168S_CMD(0xe7, 0x01),
	JD9168S_CMD(0xde, 0x00),
	JD9168S_CMD(0x35, 0x00),
};

static const u8 jd9168s_init_len[] = {
	4, 2, 3, 3, 7, 8, 3, 15, 8, 24, 10, 24,
	24, 24, 24, 38, 9, 7, 39, 2, 5, 4, 2, 2,
	3, 2, 2, 2, 2, 2,
};

static inline struct jd9168s *panel_to_jd9168s(struct drm_panel *panel)
{
	return container_of(panel, struct jd9168s, panel);
}

static int jd9168s_prepare(struct drm_panel *panel)
{
	struct jd9168s *ctx = panel_to_jd9168s(panel);
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };
	unsigned int i;

	/* The reset pin is active low: high, low, high */
	gpiod_set_value_cansleep(ctx->reset, 0);
	msleep(100);
	gpiod_set_value_cansleep(ctx->reset, 1);
	msleep(100);
	gpiod_set_value_cansleep(ctx->reset, 0);
	msleep(100);

	for (i = 0; i < ARRAY_SIZE(jd9168s_init); i++)
		mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, jd9168s_init[i],
						jd9168s_init_len[i]);

	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);

	if (dsi_ctx.accum_err) {
		gpiod_set_value_cansleep(ctx->reset, 1);
		return dsi_ctx.accum_err;
	}

	return 0;
}

static int jd9168s_enable(struct drm_panel *panel)
{
	struct jd9168s *ctx = panel_to_jd9168s(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 20);

	return dsi_ctx.accum_err;
}

static int jd9168s_disable(struct drm_panel *panel)
{
	struct jd9168s *ctx = panel_to_jd9168s(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);

	return dsi_ctx.accum_err;
}

static int jd9168s_unprepare(struct drm_panel *panel)
{
	struct jd9168s *ctx = panel_to_jd9168s(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);
	gpiod_set_value_cansleep(ctx->reset, 1);

	return 0;
}

static const struct drm_display_mode jd9168s_mode = {
	.clock		= 54000,
	.hdisplay	= 1024,
	.hsync_start	= 1024 + 40,
	.hsync_end	= 1024 + 40 + 10,
	.htotal		= 1024 + 40 + 10 + 40,
	.vdisplay	= 768,
	.vsync_start	= 768 + 12,
	.vsync_end	= 768 + 12 + 8,
	.vtotal		= 768 + 12 + 8 + 21,
	.width_mm	= 65,
	.height_mm	= 48,
	.flags		= DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
	.type		= DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static int jd9168s_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &jd9168s_mode);
}

static const struct drm_panel_funcs jd9168s_funcs = {
	.prepare	= jd9168s_prepare,
	.unprepare	= jd9168s_unprepare,
	.enable		= jd9168s_enable,
	.disable	= jd9168s_disable,
	.get_modes	= jd9168s_get_modes,
};

static int jd9168s_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct jd9168s *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct jd9168s, panel, &jd9168s_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			  MIPI_DSI_MODE_LPM;

	ctx->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset))
		return dev_err_probe(dev, PTR_ERR(ctx->reset), "getting the reset GPIO\n");

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return ret;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	return 0;
}

static void jd9168s_remove(struct mipi_dsi_device *dsi)
{
	struct jd9168s *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id jd9168s_of_match[] = {
	{ .compatible = "jadard,jd9168s" },
	{ }
};
MODULE_DEVICE_TABLE(of, jd9168s_of_match);

static struct mipi_dsi_driver jd9168s_driver = {
	.probe	= jd9168s_probe,
	.remove	= jd9168s_remove,
	.driver = {
		.name = "panel-jadard-jd9168s",
		.of_match_table = jd9168s_of_match,
	},
};
module_mipi_dsi_driver(jd9168s_driver);

MODULE_DESCRIPTION("Jadard JD9168S 1024x768 MIPI DSI panel driver");
MODULE_LICENSE("GPL");
