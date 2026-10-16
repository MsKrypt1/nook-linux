// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung LTL090CL01 9" 1920x1280 MIPI DSI panel
 *
 * The panel takes RGB666 over four DSI lanes in DE-only video mode and
 * needs no initialization commands. Besides VDD it has two control inputs:
 * CABC_EN (content adaptive brightness control) and CM_EN (color
 * management).
 *
 * Power sequence from the datasheet (LTL090CL01-W, section 5.7):
 *   T2  90..120 ms  VDD on to valid data
 *   T5  >= 200 ms   valid data to backlight on
 *   T6  >= 200 ms   backlight off to valid data off
 *   T3  0..50 ms    valid data off to VDD off
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

struct ltl090cl01 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct regulator *supply;
	struct gpio_desc *cabc_gpio;
	struct gpio_desc *cm_gpio;
};

static inline struct ltl090cl01 *to_ltl090cl01(struct drm_panel *panel)
{
	return container_of(panel, struct ltl090cl01, panel);
}

static int ltl090cl01_prepare(struct drm_panel *panel)
{
	struct ltl090cl01 *ctx = to_ltl090cl01(panel);
	int ret;

	ret = regulator_enable(ctx->supply);
	if (ret) {
		dev_err(panel->dev, "failed to enable supply: %d\n", ret);
		return ret;
	}

	gpiod_set_value_cansleep(ctx->cabc_gpio, 1);
	gpiod_set_value_cansleep(ctx->cm_gpio, 1);

	/* T2: valid data no earlier than 90 ms after VDD */
	msleep(100);

	return 0;
}

static int ltl090cl01_enable(struct drm_panel *panel)
{
	/* T5: the backlight, turned on after this, waits for valid data */
	msleep(200);

	return 0;
}

static int ltl090cl01_disable(struct drm_panel *panel)
{
	/* T6: the backlight, turned off before this, goes off before the data */
	msleep(200);

	return 0;
}

static int ltl090cl01_unprepare(struct drm_panel *panel)
{
	struct ltl090cl01 *ctx = to_ltl090cl01(panel);

	gpiod_set_value_cansleep(ctx->cm_gpio, 0);
	gpiod_set_value_cansleep(ctx->cabc_gpio, 0);

	regulator_disable(ctx->supply);

	return 0;
}

/*
 * 148.48 MHz is what the vendor firmware uses: it is exactly 4 lanes x
 * 668.16 Mbps / 18 bpp, with the DSI PLL at 1336.32 MHz from the 38.4 MHz
 * system clock (N = 20, M = 348), so the DSI and DISPC throughputs match.
 * The datasheet gives only the line length (1984 clocks typical) and DE-only
 * timing. The porches are the vendor's DSI ones: no horizontal back porch
 * (HFP 24 byte clocks), VSA 1, VBP 9, VFP 10.
 */
static const struct drm_display_mode ltl090cl01_mode = {
	.clock = 148480,
	.hdisplay = 1920,
	.hsync_start = 1920 + 43,
	.hsync_end = 1920 + 43 + 5,
	.htotal = 1920 + 43 + 5 + 0,
	.vdisplay = 1280,
	.vsync_start = 1280 + 10,
	.vsync_end = 1280 + 10 + 1,
	.vtotal = 1280 + 10 + 1 + 9,
	.width_mm = 190,
	.height_mm = 127,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static int ltl090cl01_get_modes(struct drm_panel *panel,
				struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &ltl090cl01_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	connector->display_info.bpc = 6;

	return 1;
}

static const struct drm_panel_funcs ltl090cl01_funcs = {
	.prepare = ltl090cl01_prepare,
	.enable = ltl090cl01_enable,
	.disable = ltl090cl01_disable,
	.unprepare = ltl090cl01_unprepare,
	.get_modes = ltl090cl01_get_modes,
};

static int ltl090cl01_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct ltl090cl01 *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct ltl090cl01, panel,
				   &ltl090cl01_funcs, DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->supply = devm_regulator_get(dev, "power");
	if (IS_ERR(ctx->supply))
		return dev_err_probe(dev, PTR_ERR(ctx->supply),
				     "failed to get power supply\n");

	ctx->cabc_gpio = devm_gpiod_get_optional(dev, "cabc-enable",
						 GPIOD_OUT_LOW);
	if (IS_ERR(ctx->cabc_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->cabc_gpio),
				     "failed to get cabc-enable GPIO\n");

	ctx->cm_gpio = devm_gpiod_get_optional(dev, "cm-enable", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->cm_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->cm_gpio),
				     "failed to get cm-enable GPIO\n");

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get backlight\n");

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB666_PACKED;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO;
	/* 4 x 668.16 Mbps; the LP clock period must be at least 90 ns */
	dsi->hs_rate = 334080000;
	dsi->lp_rate = 10000000;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret) {
		dev_err_probe(dev, ret, "failed to attach to DSI host\n");
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	return 0;
}

static void ltl090cl01_remove(struct mipi_dsi_device *dsi)
{
	struct ltl090cl01 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret)
		dev_err(&dsi->dev, "failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id ltl090cl01_of_match[] = {
	{ .compatible = "samsung,ltl090cl01" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ltl090cl01_of_match);

static struct mipi_dsi_driver ltl090cl01_driver = {
	.probe = ltl090cl01_probe,
	.remove = ltl090cl01_remove,
	.driver = {
		.name = "panel-samsung-ltl090cl01",
		.of_match_table = ltl090cl01_of_match,
	},
};
module_mipi_dsi_driver(ltl090cl01_driver);

MODULE_DESCRIPTION("Samsung LTL090CL01 MIPI DSI panel driver");
MODULE_LICENSE("GPL");
