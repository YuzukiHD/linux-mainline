// SPDX-License-Identifier: GPL-2.0-only
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * sunxi RTC ccu driver
 *
 * Copyright (c) 2020, DaLv <lvda@allwinnertech.com>
 */

#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/module.h>

#include "ccu_common.h"
#include "ccu_reset.h"
#include "ccu_div.h"
#include "ccu_gate.h"
#include "ccu_mp.h"
#include "ccu_mult.h"
#include "ccu_nk.h"
#include "ccu_nkm.h"
#include "ccu_nkmp.h"
#include "ccu_nm.h"
#include "ccu_phase.h"

#include "ccu-sun252i-f101-rtc.h"

/*
 * Internal OSC:
 *
 * The hardware manual does not define an enable bit for the internal
 * RC16M at XO_CTRL_REG[0]. Keep "iosc" as a 1:1 software clock alias.
 */
static CLK_FIXED_FACTOR(iosc_clk, "iosc", "rc-16m", 1, 1, 0);

static CLK_FIXED_FACTOR(iosc_div32k_clk, "iosc-div32k",
			"iosc", 500, 1, 0);

/*
 * HOSC/DCXO 24M divided 32K.
 *
 * CLK32K_FOUT_CTRL_GATING_REG[16]:
 * HOSC_TO_32K_DIVIDER_ENABLE
 */
static const char * const dcxo24M_div32k_parents[] = { "dcxo24M" };

static struct clk_init_data dcxo24M_div32k_init = {
	.name = "dcxo24M-div32k",
	.ops = &clk_fixed_rate_ops,
	.parent_names = dcxo24M_div32k_parents,
	.num_parents = ARRAY_SIZE(dcxo24M_div32k_parents),
};

static struct clk_fixed_rate dcxo24M_div32k_clk = {
	.fixed_rate = 32768,
	.hw.init = &dcxo24M_div32k_init,
};

/*
 * RTC 32K source:
 *
 * LOSC_CTRL_REG[0]:
 * 0: internal RC16M divided 32K
 * 1: DCXO/HOSC 24M divided 32K
 *
 * This is the only hardware mux controlling the RTC 32K source.
 */
static const char * const rtc32k_clk_parents[] = {
	"iosc-div32k",
	"dcxo24M-div32k",
};

static SUNXI_CCU_MUX_TABLE_WITH_GATE_FEAT(rtc32k_clk, "rtc32k",
					  rtc32k_clk_parents, NULL,
					  LOSC_CTRL_REG, 0, 1, 0, 0,
					  CCU_FEATURE_KEY_FIELD);

static CLK_FIXED_FACTOR(clk1m_rc_clk, "clk1m-rc", "rc-16m", 16, 1, 0);

static CLK_FIXED_FACTOR(rtc_1k_clk, "rtc-1k", "rtc32k", 32, 1, 0);

/*
 * RTC 32K fanout, for debug only.
 *
 * CLK32K_FOUT_CTRL_GATING_REG[2:1]:
 * 00: SYS_32K (rtc32k)
 * 01: internal OSC divided 32K
 * 10: HOSC/DCXO divided 32K
 *
 * CLK32K_FOUT_CTRL_GATING_REG[0]:
 * 1: enable fanout output
 */
static const char * const rtc_32k_fanout_clk_parents[] = {
	"rtc32k",
	"iosc-div32k",
	"dcxo24M-div32k",
};

static SUNXI_CCU_MUX_WITH_GATE(rtc_32k_fanout_clk, "rtc-32k-fanout",
				       rtc_32k_fanout_clk_parents,
				       0x60, 1, 2, BIT(0), 0);

/*
 * RTC SPI clock:
 *
 * RTC_SPI_CLK_CTRL_REG[4:0]: M divider
 * rate = psi (AHB0) / (M + 1)
 *
 * RTC_SPI_CLK_CTRL_REG[31]:
 * 0: gating
 * 1: not gating (clock enabled)
 */
static SUNXI_CCU_M_WITH_GATE(rtc_spi_clk, "rtc-spi", "psi",
			     0x310, 0, 5, 0, 0);

static struct ccu_common *sun252i_f101_rtc_ccu_clks[] = {
	&rtc32k_clk.common,
	&rtc_32k_fanout_clk.common,
	&rtc_spi_clk.common,
};

static struct clk_hw_onecell_data sun252i_f101_rtc_ccu_hw_clks = {
	.hws = {
		[CLK_RTC_IOSC] = &iosc_clk.hw,
		[CLK_RTC_IOSC_DIV32K] = &iosc_div32k_clk.hw,
		/*
		 * This SoC has no independent osc32k clock. Do not reuse rtc32k
		 * here: each clk_hw is registered exactly once by sunxi_ccu_probe().
		 */
		[CLK_RTC_DCXO24M_DIV32K] = &dcxo24M_div32k_clk.hw,
		[CLK_RTC_32K] = &rtc32k_clk.common.hw,
		[CLK_RTC_1K] = &rtc_1k_clk.hw,
		[CLK_RTC_32K_FANOUT] = &rtc_32k_fanout_clk.common.hw,
		[CLK_RTC_SPI] = &rtc_spi_clk.common.hw,
		[CLK_RTC_CLK1M_RC] = &clk1m_rc_clk.hw,
	},
	.num = CLK_RTC_NUMBER,
};

static const struct sunxi_ccu_desc sun252i_f101_rtc_ccu_desc = {
	.ccu_clks	= sun252i_f101_rtc_ccu_clks,
	.num_ccu_clks	= ARRAY_SIZE(sun252i_f101_rtc_ccu_clks),

	.hw_clks	= &sun252i_f101_rtc_ccu_hw_clks,
};

static void clock_source_init(char __iomem *base)
{
	u32 val;

	/* XO_CTRL_REG[1]: DCXO_EN = 1 */
	val = readl(base + XO_CTRL_REG);
	val |= BIT(1);
	writel(val, base + XO_CTRL_REG);

	/*
	 * CLK32K_FOUT_CTRL_GATING_REG[16]:
	 * HOSC_TO_32K_DIVIDER_ENABLE = 1
	 */
	val = readl(base + CLK32K_FOUT_CTRL_GATING_REG);
	val |= BIT(16);
	writel(val, base + CLK32K_FOUT_CTRL_GATING_REG);

	/* CLK32K_FOUT_CTRL_GATING_REG[2:1]: SYS_32K (rtc32k). */
	val &= ~GENMASK(2, 1);
	writel(val, base + CLK32K_FOUT_CTRL_GATING_REG);

	/*
	 * LOSC_CTRL_REG[0]:
	 * 0: RC16M divided 32K
	 * 1: DCXO/HOSC 24M divided 32K
	 *
	 * The key field must be written as 0x16AA together with bit[0].
	 */
	val = readl(base + LOSC_CTRL_REG);
	val &= ~BIT(0);
	val |= KEY_FIELD_MAGIC_NUM_RTC | BIT(0);
	writel(val, base + LOSC_CTRL_REG);
}

static int sun252i_f101_rtc_ccu_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct device *dev = &pdev->dev;
	void __iomem *reg;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "Fail to get IORESOURCE_MEM\n");
		return -EINVAL;
	}

	/*
	 * Don't use devm_ioremap_resource() here! Or else the RTC driver will
	 * not able to get the same resource later in rtc-sunxi.c.
	 */
	reg = devm_ioremap(dev, res->start, resource_size(res));
	if (!reg) {
		dev_err(dev, "Fail to map IO resource\n");
		return -ENOMEM;
	}

	clock_source_init(reg);

	return devm_sunxi_ccu_probe(dev, reg, &sun252i_f101_rtc_ccu_desc);
}

static const struct of_device_id sun252i_f101_rtc_ccu_ids[] = {
	{ .compatible = "allwinner,sun252i-f101-rtc-ccu" },
	{ }
};
MODULE_DEVICE_TABLE(of, sun252i_f101_rtc_ccu_ids);

static struct platform_driver sun252i_f101_rtc_ccu_driver = {
	.probe	= sun252i_f101_rtc_ccu_probe,
	.driver	= {
		.name	= "sun252i-f101-rtc-ccu",
		.of_match_table	= sun252i_f101_rtc_ccu_ids,
	},
};

module_platform_driver(sun252i_f101_rtc_ccu_driver);

MODULE_IMPORT_NS("SUNXI_CCU");
MODULE_DESCRIPTION("sunxi RTC CCU driver");
MODULE_AUTHOR("wangxinfeng <wangxinfeng@allwinnertech.com>");
MODULE_LICENSE("GPL v2");
