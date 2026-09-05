// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved.
 *
 * Allwinnertech pulse-width-modulation controller driver
 *
 * SUN252i F101 port of the Tina/BSP pwm-sunxi driver, trimmed to the
 * single-channel output path:
 *   - clock gating lives in PWM_PCGR (separate from the per-pair divider)
 *   - each channel controls its own counter (PCR/PPR) at a 0x20 stride
 * Dropped from the BSP driver: dual-channel binding, group capture, the
 * capture interrupt, AMP resource sharing, PM suspend/resume and the
 * group sysfs hooks. The standard PWM framework (pwmchip_alloc) is used.
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

/* Shared (per channel-pair) clock control; channel gating is separate */
#define PWM_PCCR01			0x20
#define PWM_PCGR			0x40

/* Channel enable register (one bit per channel) */
#define PWM_PER				0x80

/* Per-channel registers, 0x20 stride */
#define PWM_CH0_CTRL			0x100	/* PCR */
#define PWM_CH0_PERIOD			0x104	/* PPR */
#define PWM_CH_STEP			0x20

#define PWM_CLK_SRC_SHIFT		7
#define PWM_CLK_SRC_WIDTH		2
#define PWM_DIV_M_SHIFT			0
#define PWM_DIV_M_WIDTH			4
#define PWM_DIV_M_MASK			GENMASK(3, 0)

#define PWM_PRESCAL_SHIFT		0
#define PWM_PRESCAL_WIDTH		8
#define PWM_PRESCAL_K			GENMASK(7, 0)

#define PWM_ACT_STA_SHIFT		8
#define PWM_ACT_STA_WIDTH		1

#define PWM_ACT_CYCLES_SHIFT		0
#define PWM_ACT_CYCLES_WIDTH		16
#define PWM_PERIOD_CYCLES_SHIFT		16
#define PWM_PERIOD_CYCLES_WIDTH		16

#define PWM_CGR_BYPASS_SHIFT		16
#define PWM_BYPASS_WIDTH		1

#define PWM_DUTY			GENMASK(15, 0)
#define PWM_PERIOD			GENMASK(31, 16)

#define PRESCALE_MAX			256
#define PRESCALE_CYCLES_MAX		65536

#define SUNXI_PWM_CLK_100M		100000000UL
#define SUNXI_PWM_CLK_24M		24000000UL
#define SUNXI_DIV_CLK			1000000000ULL

/*
 * Clock division register value -> actual pre-divider. value N selects a
 * divider of 2^N (register value == log2 of the divider).
 */
static const u16 sunxi_pwm_pre_scal[] = { 1, 2, 4, 8, 16, 32, 64, 128, 256 };

struct sunxi_pwm_data {
	unsigned int npwm;
};

struct sunxi_pwm_chip {
	const struct sunxi_pwm_data *data;
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	struct reset_control *reset;
	spinlock_t lock;
};

static const struct sunxi_pwm_data sunxi_pwm_f101_data = {
	.npwm = 4,
};

static inline u32 sunxi_pwm_pccr(unsigned int chan)
{
	return PWM_PCCR01 + ((chan >> 1) << 2);
}

static inline u32 sunxi_pwm_chan_ctrl(unsigned int chan)
{
	return PWM_CH0_CTRL + chan * PWM_CH_STEP;
}

static inline u32 sunxi_pwm_chan_period(unsigned int chan)
{
	return PWM_CH0_PERIOD + chan * PWM_CH_STEP;
}

static inline u32 sunxi_pwm_set_bits(u32 val, u32 shift, u32 width, u32 data)
{
	u32 mask = GENMASK(shift + width - 1, shift);

	return (val & ~mask) | ((data << shift) & mask);
}

/*
 * Pick the counter source clock and configure it in the per-pair clock
 * control register. Returns the selected source frequency (Hz), or 0 when a
 * bypass path (period shorter than one 100MHz cycle) is taken.
 */
static long sunxi_pwm_config_clk(struct sunxi_pwm_chip *chip, unsigned int chan,
				 u32 period_ns)
{
	u32 pccr = sunxi_pwm_pccr(chan);
	u32 reg;
	u64 clk = SUNXI_PWM_CLK_24M;
	u32 src_sel = 0;

	if (period_ns > 0 && period_ns <= 10) {
		/* Period shorter than the 100MHz source period: bypass. */
		reg = readl(chip->base + PWM_PCGR);
		reg = sunxi_pwm_set_bits(reg, chan + PWM_CGR_BYPASS_SHIFT,
					 PWM_BYPASS_WIDTH, 1);
		writel(reg, chip->base + PWM_PCGR);

		reg = readl(chip->base + pccr);
		reg = sunxi_pwm_set_bits(reg, PWM_CLK_SRC_SHIFT,
					 PWM_CLK_SRC_WIDTH, 1);
		writel(reg, chip->base + pccr);
		return 0;
	}

	if (period_ns > 10 && period_ns <= 334) {
		/* 3MHz..100MHz range uses the 100MHz source. */
		clk = SUNXI_PWM_CLK_100M;
		src_sel = 1;
	}

	/* Never bypass for a divided source. */
	reg = readl(chip->base + PWM_PCGR);
	reg = sunxi_pwm_set_bits(reg, chan + PWM_CGR_BYPASS_SHIFT,
				 PWM_BYPASS_WIDTH, 0);
	writel(reg, chip->base + PWM_PCGR);

	reg = readl(chip->base + pccr);
	reg = sunxi_pwm_set_bits(reg, PWM_CLK_SRC_SHIFT, PWM_CLK_SRC_WIDTH,
				 src_sel);
	writel(reg, chip->base + pccr);

	return (long)clk;
}

static int sunxi_pwm_config_single(struct sunxi_pwm_chip *chip,
				   unsigned int chan, u64 duty_ns, u64 period_ns)
{
	u32 pccr = sunxi_pwm_pccr(chan);
	u32 reg, div_m = 0, prescale = 0, prescale_id;
	unsigned int entire_cycles, active_cycles;
	u64 cycles, scaled;
	long clk;

	/*
	 * The clock gating bit lives in PWM_PCGR, so clearing the whole pair
	 * control register here only drops the divider/source fields.
	 */
	writel(0, chip->base + pccr);

	clk = sunxi_pwm_config_clk(chip, chan, period_ns);
	if (clk <= 0)
		return (int)clk;

	/* Number of source-clock beats in one period. */
	cycles = (u64)clk * period_ns;
	cycles = div64_u64(cycles, SUNXI_DIV_CLK);
	entire_cycles = cycles;

	/*
	 * Scale the counter period down until it fits the 16-bit PERIOD
	 * field, using the pair divider (div_m, power of two) first and then
	 * the per-channel prescaler.
	 */
	for (prescale_id = 0; prescale_id < ARRAY_SIZE(sunxi_pwm_pre_scal);
	     prescale_id++) {
		if (entire_cycles <= PRESCALE_CYCLES_MAX)
			break;
		for (prescale = 0; prescale <= PRESCALE_MAX; prescale++) {
			scaled = div_u64(cycles, sunxi_pwm_pre_scal[prescale_id]);
			scaled = div_u64(scaled, prescale + 1);
			if (scaled <= PRESCALE_CYCLES_MAX) {
				entire_cycles = scaled;
				div_m = prescale_id;
				break;
			}
		}
	}

	if (entire_cycles == 0)
		entire_cycles = 1;

	scaled = (u64)entire_cycles * duty_ns;
	scaled = div64_u64(scaled, period_ns);
	active_cycles = scaled;

	/* pair clock control: clk_div_m */
	reg = readl(chip->base + pccr);
	reg = sunxi_pwm_set_bits(reg, PWM_DIV_M_SHIFT, PWM_DIV_M_WIDTH, div_m);
	writel(reg, chip->base + pccr);

	/* per-channel control: prescal_k */
	reg = readl(chip->base + sunxi_pwm_chan_ctrl(chan));
	reg = sunxi_pwm_set_bits(reg, PWM_PRESCAL_SHIFT, PWM_PRESCAL_WIDTH,
				 prescale);
	writel(reg, chip->base + sunxi_pwm_chan_ctrl(chan));

	/* period and duty cycles */
	reg = readl(chip->base + sunxi_pwm_chan_period(chan));
	reg = sunxi_pwm_set_bits(reg, PWM_ACT_CYCLES_SHIFT, PWM_ACT_CYCLES_WIDTH,
				 active_cycles);
	reg = sunxi_pwm_set_bits(reg, PWM_PERIOD_CYCLES_SHIFT,
				 PWM_PERIOD_CYCLES_WIDTH, entire_cycles - 1);
	writel(reg, chip->base + sunxi_pwm_chan_period(chan));

	dev_dbg(chip->dev, "chan%u: active=%u cycles=%u prescale=%u div_m=%u\n",
		chan, active_cycles, entire_cycles, prescale, div_m);
	return 0;
}

static void sunxi_pwm_set_polarity(struct sunxi_pwm_chip *chip,
				   unsigned int chan, enum pwm_polarity polarity)
{
	u32 reg;

	reg = readl(chip->base + sunxi_pwm_chan_ctrl(chan));
	reg = sunxi_pwm_set_bits(reg, PWM_ACT_STA_SHIFT, PWM_ACT_STA_WIDTH,
				 polarity == PWM_POLARITY_NORMAL);
	writel(reg, chip->base + sunxi_pwm_chan_ctrl(chan));
}

static void sunxi_pwm_enable_single(struct sunxi_pwm_chip *chip,
				    unsigned int chan)
{
	u32 reg;

	/* Start the channel counter, then un-gate its clock (BSP order). */
	reg = readl(chip->base + PWM_PER);
	reg |= BIT(chan);
	writel(reg, chip->base + PWM_PER);

	reg = readl(chip->base + PWM_PCGR);
	reg |= BIT(chan);
	writel(reg, chip->base + PWM_PCGR);
}

static void sunxi_pwm_disable_single(struct sunxi_pwm_chip *chip,
				     unsigned int chan)
{
	u32 reg;

	reg = readl(chip->base + PWM_PER);
	reg &= ~BIT(chan);
	writel(reg, chip->base + PWM_PER);

	reg = readl(chip->base + PWM_PCGR);
	reg &= ~BIT(chan);
	writel(reg, chip->base + PWM_PCGR);
}

static int sunxi_pwm_apply(struct pwm_chip *pwm_chip, struct pwm_device *pwm,
			   const struct pwm_state *state)
{
	struct sunxi_pwm_chip *chip = pwmchip_get_drvdata(pwm_chip);
	const struct pwm_state *old = &pwm->state;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&chip->lock, flags);

	if (!state->enabled) {
		if (old->enabled)
			sunxi_pwm_disable_single(chip, pwm->hwpwm);
		goto out;
	}

	/* Hardware default is "inversed", so always program the polarity. */
	sunxi_pwm_set_polarity(chip, pwm->hwpwm, state->polarity);

	ret = sunxi_pwm_config_single(chip, pwm->hwpwm,
				      state->duty_cycle, state->period);
	if (ret)
		goto out;

	if (!old->enabled)
		sunxi_pwm_enable_single(chip, pwm->hwpwm);

out:
	spin_unlock_irqrestore(&chip->lock, flags);
	return ret;
}

static int sunxi_pwm_get_state(struct pwm_chip *pwm_chip,
			       struct pwm_device *pwm, struct pwm_state *state)
{
	struct sunxi_pwm_chip *chip = pwmchip_get_drvdata(pwm_chip);
	unsigned int chan = pwm->hwpwm;
	u32 pccr, reg, prescale, period, duty;
	unsigned long flags;
	u64 clk;

	spin_lock_irqsave(&chip->lock, flags);

	reg = readl(chip->base + sunxi_pwm_chan_ctrl(chan));
	state->polarity = (reg & BIT(PWM_ACT_STA_SHIFT)) ?
		PWM_POLARITY_NORMAL : PWM_POLARITY_INVERSED;

	state->enabled = readl(chip->base + PWM_PER) & BIT(chan);

	prescale = (reg & PWM_PRESCAL_K) + 1;

	reg = readl(chip->base + sunxi_pwm_chan_period(chan));
	duty = reg & PWM_DUTY;
	period = (reg & PWM_PERIOD) >> PWM_PERIOD_CYCLES_SHIFT;

	pccr = readl(chip->base + sunxi_pwm_pccr(chan));
	if (pccr & BIT(PWM_CLK_SRC_SHIFT))
		clk = SUNXI_PWM_CLK_100M;
	else
		clk = SUNXI_PWM_CLK_24M;
	clk = div64_u64(clk, 1U << (pccr & PWM_DIV_M_MASK));

	state->period = div64_u64((u64)period * prescale * SUNXI_DIV_CLK, clk);
	state->duty_cycle = div64_u64((u64)duty * prescale * SUNXI_DIV_CLK, clk);

	spin_unlock_irqrestore(&chip->lock, flags);
	return 0;
}

static const struct pwm_ops sunxi_pwm_ops = {
	.apply = sunxi_pwm_apply,
	.get_state = sunxi_pwm_get_state,
};

static const struct of_device_id sunxi_pwm_dt_ids[] = {
	{ .compatible = "allwinner,sun252i-f101-pwm",
	  .data = &sunxi_pwm_f101_data },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sunxi_pwm_dt_ids);

static int sunxi_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct sunxi_pwm_data *data;
	struct sunxi_pwm_chip *chip;
	struct pwm_chip *pwm_chip;
	int ret;

	data = of_device_get_match_data(dev);
	if (!data)
		return -ENODEV;

	pwm_chip = devm_pwmchip_alloc(dev, data->npwm, sizeof(*chip));
	if (IS_ERR(pwm_chip))
		return PTR_ERR(pwm_chip);
	chip = pwmchip_get_drvdata(pwm_chip);

	chip->data = data;
	chip->dev = dev;
	chip->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(chip->base))
		return PTR_ERR(chip->base);

	/* BUS_PWM gate clock and its reset are needed to access the block. */
	chip->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(chip->clk))
		return dev_err_probe(dev, PTR_ERR(chip->clk),
				     "failed to get clock\n");

	chip->reset = devm_reset_control_get_optional_shared(dev, NULL);
	if (IS_ERR(chip->reset))
		return dev_err_probe(dev, PTR_ERR(chip->reset),
				     "failed to get reset control\n");

	ret = reset_control_deassert(chip->reset);
	if (ret)
		return dev_err_probe(dev, ret, "failed to deassert reset\n");

	ret = clk_prepare_enable(chip->clk);
	if (ret) {
		reset_control_assert(chip->reset);
		return dev_err_probe(dev, ret, "failed to enable clock\n");
	}

	spin_lock_init(&chip->lock);
	pwm_chip->ops = &sunxi_pwm_ops;

	ret = devm_pwmchip_add(dev, pwm_chip);
	if (ret) {
		clk_disable_unprepare(chip->clk);
		reset_control_assert(chip->reset);
		return dev_err_probe(dev, ret, "failed to add PWM chip\n");
	}

	return 0;
}

static struct platform_driver sunxi_pwm_driver = {
	.probe = sunxi_pwm_probe,
	.driver = {
		.name = "sunxi-pwm",
		.of_match_table = sunxi_pwm_dt_ids,
	},
};
module_platform_driver(sunxi_pwm_driver);

MODULE_ALIAS("platform:sunxi-pwm");
MODULE_AUTHOR("Allwinner Technology Co., Ltd.");
MODULE_DESCRIPTION("Allwinner SUN252i F101 PWM driver");
MODULE_LICENSE("GPL");

