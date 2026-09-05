// SPDX-License-Identifier: GPL-2.0-only

#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

#include <dt-bindings/clock/sun252i-f101-cpupll.h>

#include "ccu_common.h"
#include "ccu_div.h"
#include "ccu_mp.h"
#include "ccu_nkmp.h"

#define SUN252I_F101_PLL_CPU_REG	0x0000
#define SUN252I_F101_RISCV_CLK_REG	0x0d00

static struct ccu_nkmp pll_cpu_clk = {
	.enable		= BIT(31),
	.lock		= BIT(28),
	.n		= _SUNXI_CCU_MULT_OFFSET_MIN_MAX(8, 8, 0, 20, 84),
	.m		= _SUNXI_CCU_DIV(0, 1),
	.p		= _SUNXI_CCU_DIV(0, 1),
	.max_rate	= 2016000000,
	.common		= {
		.reg		= SUN252I_F101_PLL_CPU_REG,
		.hw.init	= CLK_HW_INIT("pll-cpu", "dcxo24M",
					   &ccu_nkmp_ops,
					   CLK_GET_RATE_NOCACHE | CLK_IS_CRITICAL |
					   CLK_SET_RATE_UNGATE),
	},
};

static const char * const riscv_parents[] = {
	"dcxo24M", "rtc32k", "clk1m-rc", "pll-peri-800m",
	"pll-peri-1x", "pll-cpu", "pll-audio1-div2",
};

static SUNXI_CCU_M_WITH_MUX(riscv_clk, "riscv", riscv_parents,
				SUN252I_F101_RISCV_CLK_REG, 0, 5, 24, 3,
				CLK_SET_RATE_NO_REPARENT | CLK_SET_RATE_PARENT |
				CLK_IS_CRITICAL);

static SUNXI_CCU_M(riscv_axi_clk, "riscv-axi", "riscv",
			   SUN252I_F101_RISCV_CLK_REG, 8, 2, 0);

static struct clk_hw_onecell_data sun252i_f101_cpupll_hw_clks = {
	.hws = {
		[CLK_PLL_CPU] = &pll_cpu_clk.common.hw,
		[CLK_RISCV] = &riscv_clk.common.hw,
		[CLK_RISCV_AXI] = &riscv_axi_clk.common.hw,
	},
	.num = CLK_CPUPLL_NUMBER,
};

static struct ccu_common *sun252i_f101_cpupll_clks[] = {
	&pll_cpu_clk.common,
	&riscv_clk.common,
	&riscv_axi_clk.common,
};

static const struct sunxi_ccu_desc sun252i_f101_cpupll_desc = {
	.ccu_clks = sun252i_f101_cpupll_clks,
	.num_ccu_clks = ARRAY_SIZE(sun252i_f101_cpupll_clks),
	.hw_clks = &sun252i_f101_cpupll_hw_clks,
};

static int sun252i_f101_cpupll_probe(struct platform_device *pdev)
{
	struct resource *res;
	void __iomem *reg;
	u32 val;

	/* The PLL and CPU clock selector share the main CCU register window.
	 * Do not claim the resource: the main CCU driver maps it as well.
	 */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	reg = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!reg)
		return -ENOMEM;

	val = readl(reg + SUN252I_F101_PLL_CPU_REG);
	val |= BIT(27) | BIT(29) | BIT(30) | BIT(31);
	writel(val, reg + SUN252I_F101_PLL_CPU_REG);

	val = readl(reg + SUN252I_F101_RISCV_CLK_REG);
	val &= ~GENMASK(26, 24);
	val |= 5 << 24;
	writel(val, reg + SUN252I_F101_RISCV_CLK_REG);

	of_sunxi_ccu_probe(pdev->dev.of_node, reg,
			  &sun252i_f101_cpupll_desc);
	return 0;
}

static const struct of_device_id sun252i_f101_cpupll_ids[] = {
	{ .compatible = "allwinner,sun252i-f101-cpupll" },
	{ }
};
MODULE_DEVICE_TABLE(of, sun252i_f101_cpupll_ids);

static struct platform_driver sun252i_f101_cpupll_driver = {
	.probe = sun252i_f101_cpupll_probe,
	.driver = {
		.name = "sun252i-f101-cpupll",
		.of_match_table = sun252i_f101_cpupll_ids,
	},
};
module_platform_driver(sun252i_f101_cpupll_driver);

MODULE_IMPORT_NS("SUNXI_CCU");
MODULE_DESCRIPTION("Allwinner sun252i F101 CPU PLL");
MODULE_LICENSE("GPL");
