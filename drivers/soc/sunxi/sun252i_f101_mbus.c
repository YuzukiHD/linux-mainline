// SPDX-License-Identifier: GPL-2.0-only
/*
 * Arbitration of the memory bus (MBUS) of the Allwinner F101.
 *
 * Each bus master has a priority and a bandwidth limit, and a monitor counts
 * the traffic of the groups of masters. The masters that have settings are
 * the child nodes of the device node, with the number of the master as reg.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/seq_file.h>

#define F101_MBUS_PMU			0x09c
#define F101_MBUS_PMU_CTRL		0x00
#define F101_MBUS_PMU_CTRL_EN		BIT(0)
#define F101_MBUS_PMU_CTRL_WINDOW	GENMASK(31, 16)
#define F101_MBUS_PMU_COUNTER(n)	(0x04 + (n) * 4)

#define F101_MBUS_MASTER_STRIDE		0x10
#define F101_MBUS_MSC(m)		(0x210 + (m) * F101_MBUS_MASTER_STRIDE)
#define F101_MBUS_MSC_PRIORITY		GENMASK(3, 2)
#define F101_MBUS_BWLR(m)		(0x218 + (m) * F101_MBUS_MASTER_STRIDE)
#define F101_MBUS_BWLR_LIMIT		GENMASK(27, 16)
#define F101_MBUS_BWLR_EN		BIT(31)

#define F101_MBUS_MASTERS		40

static const char * const f101_mbus_counters[] = {
	"cpu", "rv_sys", "mahb", "dma", "ve", "ce", "tvd", "csi",
	"dsp_sys", "g2d", "di", "de", "iommu", "reserved", "other", "total",
};

struct f101_mbus {
	struct device *dev;
	void __iomem *base;
	unsigned long clk_mhz;
};

static void f101_mbus_set_priority(struct f101_mbus *mbus, u32 master, u32 prio)
{
	u32 reg = readl(mbus->base + F101_MBUS_MSC(master));

	reg &= ~F101_MBUS_MSC_PRIORITY;
	reg |= FIELD_PREP(F101_MBUS_MSC_PRIORITY, prio);
	writel(reg, mbus->base + F101_MBUS_MSC(master));
}

/* The limit register counts 1/256 byte per MBUS clock cycle, and 0 mbps is no limit */
static void f101_mbus_set_limit(struct f101_mbus *mbus, u32 master, u32 mbps)
{
	u32 reg = readl(mbus->base + F101_MBUS_BWLR(master));
	u32 limit;

	reg &= ~(F101_MBUS_BWLR_LIMIT | F101_MBUS_BWLR_EN);
	if (mbps) {
		limit = min_t(u64, div_u64(256ULL * mbps, mbus->clk_mhz),
			      FIELD_MAX(F101_MBUS_BWLR_LIMIT));
		reg |= FIELD_PREP(F101_MBUS_BWLR_LIMIT, limit) | F101_MBUS_BWLR_EN;
	}
	writel(reg, mbus->base + F101_MBUS_BWLR(master));
}

static int f101_mbus_traffic_show(struct seq_file *s, void *unused)
{
	struct f101_mbus *mbus = s->private;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(f101_mbus_counters); i++)
		seq_printf(s, "%-8s %u\n", f101_mbus_counters[i],
			   readl(mbus->base + F101_MBUS_PMU + F101_MBUS_PMU_COUNTER(i)));

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(f101_mbus_traffic);

static int f101_mbus_masters_show(struct seq_file *s, void *unused)
{
	struct f101_mbus *mbus = s->private;
	unsigned int m;

	for (m = 0; m < F101_MBUS_MASTERS; m++) {
		u32 msc = readl(mbus->base + F101_MBUS_MSC(m));
		u32 bwlr = readl(mbus->base + F101_MBUS_BWLR(m));

		seq_printf(s, "%2u priority %lu limit ", m,
			   FIELD_GET(F101_MBUS_MSC_PRIORITY, msc));
		if (bwlr & F101_MBUS_BWLR_EN)
			seq_printf(s, "%lu MB/s\n",
				   FIELD_GET(F101_MBUS_BWLR_LIMIT, bwlr) *
				   mbus->clk_mhz / 256);
		else
			seq_puts(s, "none\n");
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(f101_mbus_masters);

static int f101_mbus_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct f101_mbus *mbus;
	struct dentry *dir;
	struct clk *clk, *bus;
	struct reset_control *rst;
	u32 window = 50000;
	int ret;

	mbus = devm_kzalloc(dev, sizeof(*mbus), GFP_KERNEL);
	if (!mbus)
		return -ENOMEM;
	mbus->dev = dev;

	mbus->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(mbus->base))
		return PTR_ERR(mbus->base);

	rst = devm_reset_control_get_optional_exclusive_deasserted(dev, NULL);
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst), "cannot get reset\n");

	bus = devm_clk_get_enabled(dev, "bus");
	if (IS_ERR(bus))
		return dev_err_probe(dev, PTR_ERR(bus), "getting the bus clock\n");

	clk = devm_clk_get_enabled(dev, "mbus");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "getting the MBUS clock\n");

	mbus->clk_mhz = clk_get_rate(clk) / 1000000;
	if (!mbus->clk_mhz)
		return dev_err_probe(dev, -EINVAL, "the MBUS clock has no rate\n");

	/* Start the traffic monitor */
	of_property_read_u32(dev->of_node, "allwinner,monitor-window-us", &window);
	writel(0, mbus->base + F101_MBUS_PMU + F101_MBUS_PMU_CTRL);
	writel(FIELD_PREP(F101_MBUS_PMU_CTRL_WINDOW, window) | F101_MBUS_PMU_CTRL_EN,
	       mbus->base + F101_MBUS_PMU + F101_MBUS_PMU_CTRL);

	for_each_available_child_of_node_scoped(dev->of_node, child) {
		u32 master, value;

		ret = of_property_read_u32(child, "reg", &master);
		if (ret || master >= F101_MBUS_MASTERS) {
			dev_warn(dev, "%pOF: no valid master\n", child);
			continue;
		}

		if (!of_property_read_u32(child, "priority", &value)) {
			if (value > 3)
				dev_warn(dev, "%pOF: priority over 3\n", child);
			else
				f101_mbus_set_priority(mbus, master, value);
		}

		if (!of_property_read_u32(child, "bandwidth-limit-mbps", &value))
			f101_mbus_set_limit(mbus, master, value);
	}

	dir = debugfs_create_dir("sun252i-f101-mbus", NULL);
	debugfs_create_file("traffic", 0444, dir, mbus, &f101_mbus_traffic_fops);
	debugfs_create_file("masters", 0444, dir, mbus, &f101_mbus_masters_fops);
	platform_set_drvdata(pdev, dir);

	dev_info(dev, "MBUS clock %lu MHz\n", mbus->clk_mhz);

	return 0;
}

static void f101_mbus_remove(struct platform_device *pdev)
{
	debugfs_remove_recursive(platform_get_drvdata(pdev));
}

static const struct of_device_id f101_mbus_of_match[] = {
	{ .compatible = "allwinner,sun252i-f101-mbus" },
	{ }
};
MODULE_DEVICE_TABLE(of, f101_mbus_of_match);

static struct platform_driver f101_mbus_driver = {
	.probe = f101_mbus_probe,
	.remove = f101_mbus_remove,
	.driver = {
		.name = "sun252i-f101-mbus",
		.of_match_table = f101_mbus_of_match,
	},
};
module_platform_driver(f101_mbus_driver);

MODULE_DESCRIPTION("Allwinner F101 memory bus arbitration");
MODULE_LICENSE("GPL");
