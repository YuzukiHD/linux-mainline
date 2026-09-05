// SPDX-License-Identifier: GPL-2.0-only

#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

#include <dt-bindings/clock/sun252i-f101-ddrpll.h>

#include "ccu_common.h"
#include "ccu_div.h"
#include "ccu_mp.h"
#include "ccu_nkmp.h"

#define SUN252I_F101_PLL_DDR_REG	0x0010
#define SUN252I_F101_DRAM_CLK_REG	0x0800

static struct ccu_nkmp pll_ddr_clk = {
	.enable		= BIT(31),
	.lock		= BIT(28),
	.n		= _SUNXI_CCU_MULT_OFFSET_MIN_MAX(8, 8, 1, 20, 84),
	.m		= _SUNXI_CCU_DIV(0, 1),
	.p		= _SUNXI_CCU_DIV(0, 1),
	.common		= {
		.reg		= SUN252I_F101_PLL_DDR_REG,
		.hw.init	= CLK_HW_INIT("pll-ddr", "dcxo24M",
					   &ccu_nkmp_ops,
					   CLK_GET_RATE_NOCACHE | CLK_IS_CRITICAL),
	},
};

static const char * const dram_parents[] = {
	"pll-peri-2x", "pll-ddr", "pll-peri-800m", "pll-audio1-div2",
};

static struct ccu_mp dram_mux_clk = {
	.enable = BIT(31),
	.m = _SUNXI_CCU_DIV(0, 2),
	.p = _SUNXI_CCU_DIV(8, 2),
	.mux = _SUNXI_CCU_MUX(24, 3),
	.common = {
		.reg = SUN252I_F101_DRAM_CLK_REG,
		.features = CCU_FEATURE_DUAL_DIV,
		.hw.init = CLK_HW_INIT_PARENTS("dram-mux-clk", dram_parents,
						      &ccu_mp_ops,
						      CLK_GET_RATE_NOCACHE |
						      CLK_IGNORE_UNUSED),
	},
};

static CLK_FIXED_FACTOR(dfi_clk, "dfi-clk", "dram-mux-clk", 2, 1, 0);

static struct clk_hw_onecell_data sun252i_f101_ddrpll_hw_clks = {
	.hws = {
		[CLK_PLL_DDR] = &pll_ddr_clk.common.hw,
		[CLK_DRAM_MUX] = &dram_mux_clk.common.hw,
		[CLK_DFI] = &dfi_clk.hw,
	},
	.num = CLK_DDRPLL_NUMBER,
};

static struct ccu_common *sun252i_f101_ddrpll_clks[] = {
	&pll_ddr_clk.common,
	&dram_mux_clk.common,
};

static const struct sunxi_ccu_desc sun252i_f101_ddrpll_desc = {
	.ccu_clks = sun252i_f101_ddrpll_clks,
	.num_ccu_clks = ARRAY_SIZE(sun252i_f101_ddrpll_clks),
	.hw_clks = &sun252i_f101_ddrpll_hw_clks,
};

static int sun252i_f101_ddrpll_probe(struct platform_device *pdev)
{
	struct resource *res;
	void __iomem *reg;

	/* The DDR PLL shares the main CCU register window.  Map it without
	 * claiming the resource, which is owned by the main CCU driver.
	 */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	reg = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!reg)
		return -ENOMEM;

	of_sunxi_ccu_probe(pdev->dev.of_node, reg,
			  &sun252i_f101_ddrpll_desc);
	return 0;
}

static const struct of_device_id sun252i_f101_ddrpll_ids[] = {
	{ .compatible = "allwinner,sun252i-f101-ddrpll" },
	{ }
};
MODULE_DEVICE_TABLE(of, sun252i_f101_ddrpll_ids);

static struct platform_driver sun252i_f101_ddrpll_driver = {
	.probe = sun252i_f101_ddrpll_probe,
	.driver = {
		.name = "sun252i-f101-ddrpll",
		.of_match_table = sun252i_f101_ddrpll_ids,
	},
};
module_platform_driver(sun252i_f101_ddrpll_driver);

MODULE_IMPORT_NS("SUNXI_CCU");
MODULE_DESCRIPTION("Allwinner sun252i F101 DDR PLL");
MODULE_LICENSE("GPL");
