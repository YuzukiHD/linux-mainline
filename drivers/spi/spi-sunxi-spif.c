// SPDX-License-Identifier: GPL-2.0-only
/*
 * Allwinner dedicated SPI flash controller (SPIF).
 *
 * Based on the V861 driver by Han Gao <gaohan@iscas.ac.cn>. Every command
 * is one descriptor that the controller's own DMA fetches, it carries the
 * command, address, dummy and data phases. The F101 controller has no
 * interrupt line, the end of the command is polled.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>

#define SPIF_VERSION		0x00
#define SPIF_GCR		0x04
#define SPIF_GAR		0x08
#define SPIF_TCR		0x0c
#define SPIF_IER		0x14
#define SPIF_ISR		0x18
#define SPIF_CSD		0x1c
#define SPIF_PHC		0x20
#define SPIF_TCF		0x24
#define SPIF_TCS		0x28
#define SPIF_TNM		0x2c
#define SPIF_DMA_CTL		0x40
#define SPIF_DMA_DESC		0x44

#define SPIF_VERSION_V2		0x10002

#define SPIF_GCR_DMA_MODE	BIT(0)
#define SPIF_GCR_NORMAL_EN	BIT(2)	/* start a command without data */
#define SPIF_GCR_DTR_EN		BIT(16)	/* address and data on both edges */
#define SPIF_GCR_CPHA		BIT(4)
#define SPIF_GCR_CPOL		BIT(5)
#define SPIF_GCR_CS		GENMASK(7, 6)
#define SPIF_GCR_CS_ACTIVE_LOW	BIT(8)

#define SPIF_GAR_FIFO_RESET	GENMASK(1, 0)
#define SPIF_GAR_SOFT_RESET	BIT(3)
#define SPIF_GAR_DMA_RESET	BIT(4)

#define SPIF_ISR_DMA_DONE	BIT(24)
#define SPIF_ISR_ERRORS		GENMASK(10, 8)
#define SPIF_ISR_ALL		(SPIF_ISR_DMA_DONE | SPIF_ISR_ERRORS)

#define SPIF_TCR_DELAY		GENMASK(5, 0)
#define SPIF_TCR_DELAY_SW_EN	BIT(6)
#define SPIF_TCR_MODE		GENMASK(18, 16)
#define SPIF_TCR_SAMPLE_EN	BIT(20)
#define SPIF_TCR_SCKOUT_SEL	BIT(26)	/* DTR clock output */
#define SPIF_TCR_SAMPLE_MASK	(SPIF_TCR_DELAY | SPIF_TCR_DELAY_SW_EN | \
				 SPIF_TCR_MODE | SPIF_TCR_SAMPLE_EN)

#define SPIF_CSD_DEFAULT	((5 << 16) | (6 << 8) | 6)

/* Descriptor word 0: burst and direction */
#define SPIF_DESC_LAST		BIT(0)
#define SPIF_DESC_READ		BIT(1)	/* the DMA writes memory */
#define SPIF_DESC_BURST		GENMASK(6, 4)
#define SPIF_DESC_BURST_INCR16	7
/* Descriptor word 1: block size and length */
#define SPIF_DESC_BLOCK		GENMASK(25, 24)
#define SPIF_DESC_BLOCK_64B	3

/* Phases of the command (descriptor word 4, PHC) */
#define SPIF_PHASE_RX		BIT(8)
#define SPIF_PHASE_TX		BIT(12)
#define SPIF_PHASE_DUMMY	BIT(16)
#define SPIF_PHASE_ADDR		BIT(24)
#define SPIF_PHASE_CMD		BIT(28)

/* Bus widths and opcode (descriptor word 6, TCS) */
#define SPIF_TCS_DATA		GENMASK(1, 0)
#define SPIF_TCS_ADDR		GENMASK(5, 4)
#define SPIF_TCS_OPCODE		GENMASK(31, 24)

/* Counts (descriptor word 7, TNM) */
#define SPIF_TNM_LEN		GENMASK(15, 0)
#define SPIF_TNM_DUMMY		GENMASK(23, 16)	/* dummy clock cycles */
#define SPIF_TNM_ADDR_LEN	GENMASK(25, 24)	/* address bytes - 1 */
#define SPIF_TNM_NORMAL		BIT(28)
#define SPIF_TNM_LEN_64K	BIT(31)

#define SPIF_DMA_START		BIT(0)
#define SPIF_DMA_DESC_LEN	GENMASK(15, 4)

#define SPIF_MAX_SPEED		100000000
/* Up to here the data is sampled right without a delay */
#define SPIF_SAFE_SPEED		50000000
#define SPIF_IDENT_SPEED	24000000
#define SPIF_TUNE_SIZE		SZ_4K
#define SPIF_TUNE_MODES		3
#define SPIF_TUNE_DELAYS	64
#define SPIF_TUNE_MIN_WINDOW	6
#define SPIF_BULK_READ		256
#define SPIF_BUFFER_SIZE	SZ_64K
#define SPIF_TIMEOUT_US		1000000

struct sunxi_spif_desc {
	__le32 burst;
	__le32 block_len;
	__le32 data_addr;
	__le32 next_desc;
	__le32 phase;
	__le32 flash_addr;
	__le32 tcs;
	__le32 tnm;
};

/* The sample point found for one kind of bulk read at one speed */
struct sunxi_spif_tune {
	u32 key;
	unsigned int speed;
	bool valid;
	bool failed;
	u32 sample;	/* SPIF_TCR_SAMPLE_* bits */
};

struct sunxi_spif {
	struct device *dev;
	void __iomem *base;
	struct clk *bus_clk;
	struct clk *mod_clk;
	struct reset_control *rst;
	struct sunxi_spif_desc *desc;
	dma_addr_t desc_dma;
	u8 *buffer;
	unsigned int speed;	/* SCK, the module clock is twice that in DTR */
	bool dtr;
	bool clocks_on;
	bool failed;

	/* Sample point tuning, needs a readable place of the flash */
	bool can_tune;
	u32 tune_offset;
	u8 *tune_ref;
	u8 *tune_buf;
	struct sunxi_spif_tune tune;
	u32 sample;	/* what SPIF_TCR holds of the sample point now */
};

/* 1, 2 and 4 wires are coded 0, 1 and 2 */
static u32 sunxi_spif_width(u8 buswidth)
{
	return buswidth == 4 ? 2 : buswidth == 2 ? 1 : 0;
}

static int sunxi_spif_reset(struct sunxi_spif *spif)
{
	u32 value;
	int ret;

	writel(readl(spif->base + SPIF_GAR) | SPIF_GAR_DMA_RESET,
	       spif->base + SPIF_GAR);
	ret = readl_poll_timeout(spif->base + SPIF_GAR, value,
				 !(value & SPIF_GAR_DMA_RESET), 1, 100000);
	if (ret)
		return ret;

	writel(readl(spif->base + SPIF_GAR) | SPIF_GAR_SOFT_RESET,
	       spif->base + SPIF_GAR);
	return readl_poll_timeout(spif->base + SPIF_GAR, value,
				  !(value & SPIF_GAR_SOFT_RESET), 1, 100000);
}

static void sunxi_spif_disable(void *data)
{
	struct sunxi_spif *spif = data;

	if (!spif->clocks_on)
		return;

	reset_control_assert(spif->rst);
	clk_disable_unprepare(spif->mod_clk);
	clk_disable_unprepare(spif->bus_clk);
	spif->clocks_on = false;
}

/* Clocks of the dummy phase, a DTR bit takes half a clock */
static unsigned int sunxi_spif_dummy_cycles(const struct spi_mem_op *op)
{
	return op->dummy.nbytes * 8 / op->dummy.buswidth / (op->dummy.dtr ? 2 : 1);
}

static bool sunxi_spif_supports_op(struct spi_mem *mem,
				   const struct spi_mem_op *op)
{
	if (op->cmd.nbytes != 1 || op->cmd.buswidth != 1)
		return false;

	/*
	 * DTR is for the whole of address, dummy and data, as 1S-4D-4D. The
	 * controller sends the opcode on one edge whatever cmd.dtr says.
	 */
	if (op->addr.dtr || op->dummy.dtr || op->data.dtr) {
		if (!op->addr.nbytes || !op->addr.dtr || !op->data.dtr ||
		    (op->dummy.nbytes && !op->dummy.dtr))
			return false;
	}

	if (op->addr.nbytes > 4 || op->addr.val > U32_MAX ||
	    (op->addr.nbytes && op->addr.buswidth > 4))
		return false;

	/* The dummy phase counts clocks, 8 bits wide */
	if (op->dummy.nbytes &&
	    sunxi_spif_dummy_cycles(op) > 255)
		return false;

	return spi_mem_default_supports_op(mem, op);
}

static int sunxi_spif_adjust_op_size(struct spi_mem *mem,
				     struct spi_mem_op *op)
{
	op->data.nbytes = min_t(unsigned int, op->data.nbytes,
				SPIF_BUFFER_SIZE);
	return 0;
}

static int sunxi_spif_set_mode(struct sunxi_spif *spif, unsigned int speed,
			       bool dtr)
{
	unsigned int rate = dtr ? 2 * speed : speed;
	int ret;

	if (speed == spif->speed && dtr == spif->dtr)
		return 0;

	ret = clk_set_rate(spif->mod_clk, rate);
	if (ret)
		return ret;
	if (clk_get_rate(spif->mod_clk) > rate)
		return -ERANGE;

	if (dtr != spif->dtr) {
		u32 value = readl(spif->base + SPIF_TCR);

		writel(dtr ? value | SPIF_TCR_SCKOUT_SEL :
			     value & ~SPIF_TCR_SCKOUT_SEL, spif->base + SPIF_TCR);
		/* The controller takes the new clocking after a soft reset */
		ret = sunxi_spif_reset(spif);
		if (ret)
			return ret;
		value = (dtr ? SPIF_TCR_SCKOUT_SEL : 0) | spif->sample;
		writel(value, spif->base + SPIF_TCR);
	}

	spif->speed = speed;
	spif->dtr = dtr;
	return 0;
}

static int sunxi_spif_xfer(struct spi_mem *mem, const struct spi_mem_op *op,
			   unsigned int speed)
{
	struct sunxi_spif *spif = spi_controller_get_devdata(mem->spi->controller);
	struct sunxi_spif_desc *desc = spif->desc;
	enum dma_data_direction direction = op->data.dir == SPI_MEM_DATA_IN ?
		DMA_FROM_DEVICE : DMA_TO_DEVICE;
	dma_addr_t buffer = 0;
	size_t mapped = round_up(op->data.nbytes, 64);
	/* The DMA moves whole 64 byte blocks, so map a whole number of them */
	void *dma_buf = spif->buffer;
	u32 burst, phase, tcs, tnm, control, status;
	int ret;

	if (spif->failed)
		return -EIO;
	if (!sunxi_spif_supports_op(mem, op) || op->data.nbytes > SPIF_BUFFER_SIZE)
		return -EOPNOTSUPP;

	ret = sunxi_spif_set_mode(spif, speed, op->data.dtr);
	if (ret)
		return ret;

	burst = SPIF_DESC_LAST |
		FIELD_PREP(SPIF_DESC_BURST, SPIF_DESC_BURST_INCR16);
	phase = SPIF_PHASE_CMD;
	tcs = FIELD_PREP(SPIF_TCS_OPCODE, op->cmd.opcode);
	tnm = SPIF_TNM_NORMAL;

	desc->flash_addr = 0;
	if (op->addr.nbytes) {
		phase |= SPIF_PHASE_ADDR;
		desc->flash_addr = cpu_to_le32(op->addr.val);
		tcs |= FIELD_PREP(SPIF_TCS_ADDR, sunxi_spif_width(op->addr.buswidth));
		tnm |= FIELD_PREP(SPIF_TNM_ADDR_LEN, op->addr.nbytes - 1);
	}
	if (op->dummy.nbytes) {
		phase |= SPIF_PHASE_DUMMY;
		tnm |= FIELD_PREP(SPIF_TNM_DUMMY, sunxi_spif_dummy_cycles(op));
	}

	desc->data_addr = 0;
	if (op->data.nbytes) {
		tcs |= FIELD_PREP(SPIF_TCS_DATA, sunxi_spif_width(op->data.buswidth));
		tnm |= op->data.nbytes == SZ_64K ? SPIF_TNM_LEN_64K :
		       FIELD_PREP(SPIF_TNM_LEN, op->data.nbytes);

		if (op->data.dir == SPI_MEM_DATA_IN) {
			phase |= SPIF_PHASE_RX;
			burst |= SPIF_DESC_READ;
		} else {
			phase |= SPIF_PHASE_TX;
			memcpy(spif->buffer, op->data.buf.out, op->data.nbytes);
		}

		buffer = dma_map_single(spif->dev, dma_buf, mapped, direction);
		if (dma_mapping_error(spif->dev, buffer))
			return -ENOMEM;
		desc->data_addr = cpu_to_le32(buffer >> 2);
	}

	desc->burst = cpu_to_le32(burst);
	desc->block_len = cpu_to_le32(FIELD_PREP(SPIF_DESC_BLOCK, SPIF_DESC_BLOCK_64B) |
				      op->data.nbytes);
	desc->next_desc = 0;
	desc->phase = cpu_to_le32(phase);
	desc->tcs = cpu_to_le32(tcs);
	desc->tnm = cpu_to_le32(tnm);
	/* The descriptor is in coherent memory, make sure it is written first */
	dma_wmb();

	control = SPIF_GCR_DMA_MODE | SPIF_GCR_CS_ACTIVE_LOW |
		  FIELD_PREP(SPIF_GCR_CS, spi_get_chipselect(mem->spi, 0));
	if (spif->dtr)
		control |= SPIF_GCR_DTR_EN;
	if (mem->spi->mode & SPI_CPHA)
		control |= SPIF_GCR_CPHA;
	if (mem->spi->mode & SPI_CPOL)
		control |= SPIF_GCR_CPOL;

	writel(readl(spif->base + SPIF_GAR) | SPIF_GAR_FIFO_RESET,
	       spif->base + SPIF_GAR);

	if (!op->data.nbytes) {
		/* No data: the command registers are written directly */
		control &= ~SPIF_GCR_DMA_MODE;
		writel(control, spif->base + SPIF_GCR);
		writel(phase, spif->base + SPIF_PHC);
		writel(op->addr.nbytes ? op->addr.val : 0, spif->base + SPIF_TCF);
		writel(tcs, spif->base + SPIF_TCS);
		writel(tnm, spif->base + SPIF_TNM);
		writel(control | SPIF_GCR_NORMAL_EN, spif->base + SPIF_GCR);

		ret = readl_poll_timeout_atomic(spif->base + SPIF_GCR, status,
						!(status & SPIF_GCR_NORMAL_EN), 1,
						SPIF_TIMEOUT_US);
	} else {
		writel(control, spif->base + SPIF_GCR);
		writel(SPIF_ISR_ALL, spif->base + SPIF_ISR);
		writel(spif->desc_dma >> 2, spif->base + SPIF_DMA_DESC);
		writel(FIELD_PREP(SPIF_DMA_DESC_LEN, sizeof(*desc)) | SPIF_DMA_START,
		       spif->base + SPIF_DMA_CTL);

		/*
		 * A transfer is short, much shorter than a sleep and the wake
		 * up that follows it: wait for it without sleeping.
		 */
		ret = readl_poll_timeout_atomic(spif->base + SPIF_ISR, status,
						status & SPIF_ISR_ALL, 1,
						SPIF_TIMEOUT_US);
		if (!ret && (status & SPIF_ISR_ERRORS))
			ret = -EIO;
		writel(status, spif->base + SPIF_ISR);
	}

	if (ret && sunxi_spif_reset(spif)) {
		/* Do not hand a live DMA buffer back to the caller. */
		sunxi_spif_disable(spif);
		spif->failed = true;
	}

	if (op->data.nbytes)
		dma_unmap_single(spif->dev, buffer, mapped, direction);
	if (!ret && op->data.nbytes && op->data.dir == SPI_MEM_DATA_IN)
		memcpy(op->data.buf.in, spif->buffer, op->data.nbytes);

	return ret;
}


static int sunxi_spif_set_sample(struct sunxi_spif *spif, u32 sample)
{
	u32 value;

	if (sample == spif->sample)
		return 0;

	value = (spif->dtr ? SPIF_TCR_SCKOUT_SEL : 0) | sample;
	writel(value, spif->base + SPIF_TCR);
	spif->sample = sample;
	/* The delay line needs a moment to settle */
	usleep_range(1000, 1500);
	return 0;
}

/* A byte pattern with enough different bytes to tell a right read from a stuck one */
static bool sunxi_spif_tune_data_ok(const u8 *data)
{
	DECLARE_BITMAP(seen, 256) = { };
	int i;

	for (i = 0; i < SPIF_TUNE_SIZE; i++)
		__set_bit(data[i], seen);

	return bitmap_weight(seen, 256) >= 32;
}

static u32 sunxi_spif_tune_key(const struct spi_mem_op *op)
{
	return op->cmd.opcode | (op->addr.buswidth << 8) | (op->data.buswidth << 12) |
	       (op->addr.nbytes << 16) | (op->dummy.nbytes << 20) | (op->data.dtr << 31);
}

/*
 * Try every sample mode and delay with the read that is going to be used,
 * against the data the same read gives at a speed that needs no tuning.
 */
static int sunxi_spif_tune_run(struct sunxi_spif *spif, struct spi_mem *mem,
			       const struct spi_mem_op *op, unsigned int speed)
{
	struct spi_mem_op probe = *op;
	unsigned int best_len = 0, best_start = 0, best_mode = 0;
	unsigned int mode, delay;
	int ret;

	probe.addr.val = spif->tune_offset;
	probe.data.nbytes = SPIF_TUNE_SIZE;
	probe.data.buf.in = spif->tune_buf;

	ret = sunxi_spif_set_sample(spif, 0);
	if (ret)
		return ret;
	ret = sunxi_spif_xfer(mem, &probe, SPIF_IDENT_SPEED);
	if (ret)
		return ret;
	memcpy(spif->tune_ref, spif->tune_buf, SPIF_TUNE_SIZE);
	if (!sunxi_spif_tune_data_ok(spif->tune_ref)) {
		dev_warn(spif->dev, "no usable data at the tune offset %#x\n",
			 spif->tune_offset);
		return -ENODATA;
	}

	for (mode = 0; mode < SPIF_TUNE_MODES; mode++) {
		unsigned int run = 0;

		for (delay = 0; delay < SPIF_TUNE_DELAYS; delay++) {
			bool good = true;
			int pass;

			sunxi_spif_set_sample(spif, SPIF_TCR_SAMPLE_EN | SPIF_TCR_DELAY_SW_EN |
					      FIELD_PREP(SPIF_TCR_MODE, mode) |
					      FIELD_PREP(SPIF_TCR_DELAY, delay));
			for (pass = 0; good && pass < 2; pass++) {
				memset(spif->tune_buf, 0, SPIF_TUNE_SIZE);
				good = !sunxi_spif_xfer(mem, &probe, speed) &&
				       !memcmp(spif->tune_buf, spif->tune_ref, SPIF_TUNE_SIZE);
			}
			if (good) {
				run++;
				if (run > best_len) {
					best_len = run;
					best_start = delay + 1 - run;
					best_mode = mode;
				}
			} else {
				run = 0;
			}
		}
	}

	if (best_len < SPIF_TUNE_MIN_WINDOW) {
		sunxi_spif_set_sample(spif, 0);
		return -EIO;
	}

	spif->tune.sample = SPIF_TCR_SAMPLE_EN | SPIF_TCR_DELAY_SW_EN |
			    FIELD_PREP(SPIF_TCR_MODE, best_mode) |
			    FIELD_PREP(SPIF_TCR_DELAY, best_start + best_len / 2);
	dev_info(spif->dev, "%u MHz%s: sample mode %u, delay %u (window %u..%u)\n",
		 speed / 1000000, op->data.dtr ? " DTR" : "", best_mode,
		 best_start + best_len / 2, best_start, best_start + best_len - 1);
	return 0;
}

/* What speed this op can run at, and with which sample point */
static int sunxi_spif_exec_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct sunxi_spif *spif = spi_controller_get_devdata(mem->spi->controller);
	unsigned int speed = mem->spi->max_speed_hz;
	bool bulk = op->data.dir == SPI_MEM_DATA_IN &&
		    (op->data.nbytes >= SPIF_BULK_READ || op->data.dtr);
	u32 key = sunxi_spif_tune_key(op);
	int ret;

	if (op->max_freq)
		speed = min(speed, op->max_freq);
	if (!speed)
		return -EINVAL;
	speed = min_t(unsigned int, speed, SPIF_MAX_SPEED);

	if (speed > SPIF_SAFE_SPEED) {
		if (!bulk || !spif->can_tune || (spif->tune.failed && spif->tune.key == key &&
						 spif->tune.speed == speed)) {
			speed = SPIF_SAFE_SPEED;
		} else if (!spif->tune.valid || spif->tune.key != key || spif->tune.speed != speed) {
			spif->tune.valid = false;
			spif->tune.key = key;
			spif->tune.speed = speed;
			spif->tune.failed = false;
			ret = sunxi_spif_tune_run(spif, mem, op, speed);
			if (ret) {
				spif->tune.failed = true;
				dev_warn(spif->dev,
					 "no sample point at %u MHz (%d), using %u MHz\n",
					 speed / 1000000, ret, SPIF_SAFE_SPEED / 1000000);
				speed = SPIF_SAFE_SPEED;
			} else {
				spif->tune.valid = true;
			}
		}
	}

	if (speed > SPIF_SAFE_SPEED) {
		sunxi_spif_set_sample(spif, spif->tune.sample);
	} else {
		sunxi_spif_set_sample(spif, 0);
	}

	return sunxi_spif_xfer(mem, op, speed);
}

static const struct spi_controller_mem_caps sunxi_spif_mem_caps = {
	.dtr = true,
};

static const struct spi_controller_mem_ops sunxi_spif_mem_ops = {
	.supports_op = sunxi_spif_supports_op,
	.adjust_op_size = sunxi_spif_adjust_op_size,
	.exec_op = sunxi_spif_exec_op,
};

static int sunxi_spif_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spi_controller *ctlr;
	struct sunxi_spif *spif;
	int ret;

	ctlr = devm_spi_alloc_host(dev, sizeof(*spif));
	if (!ctlr)
		return -ENOMEM;
	spif = spi_controller_get_devdata(ctlr);
	spif->dev = dev;

	spif->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(spif->base))
		return PTR_ERR(spif->base);

	spif->bus_clk = devm_clk_get(dev, "ahb");
	if (IS_ERR(spif->bus_clk))
		return dev_err_probe(dev, PTR_ERR(spif->bus_clk), "getting bus clock\n");

	spif->mod_clk = devm_clk_get(dev, "mod");
	if (IS_ERR(spif->mod_clk))
		return dev_err_probe(dev, PTR_ERR(spif->mod_clk), "getting module clock\n");

	spif->rst = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(spif->rst))
		return dev_err_probe(dev, PTR_ERR(spif->rst), "getting reset\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	spif->desc = dmam_alloc_coherent(dev, sizeof(*spif->desc),
					 &spif->desc_dma, GFP_KERNEL);
	/* A whole number of cache lines: the DMA writes whole 64 byte blocks */
	spif->buffer = devm_kmalloc(dev, SPIF_BUFFER_SIZE, GFP_KERNEL);
	if (!spif->desc || !spif->buffer)
		return -ENOMEM;

	ret = clk_prepare_enable(spif->bus_clk);
	if (ret)
		return ret;
	ret = clk_prepare_enable(spif->mod_clk);
	if (ret) {
		clk_disable_unprepare(spif->bus_clk);
		return ret;
	}
	spif->clocks_on = true;
	ret = devm_add_action_or_reset(dev, sunxi_spif_disable, spif);
	if (ret)
		return ret;

	ret = reset_control_deassert(spif->rst);
	if (ret)
		return ret;

	writel(0, spif->base + SPIF_IER);
	ret = sunxi_spif_reset(spif);
	if (ret)
		return dev_err_probe(dev, ret, "resetting the controller\n");

	if (readl(spif->base + SPIF_VERSION) < SPIF_VERSION_V2)
		return dev_err_probe(dev, -ENODEV, "unsupported controller version %#x\n",
				     readl(spif->base + SPIF_VERSION));

	writel(0, spif->base + SPIF_TCR);
	writel(SPIF_CSD_DEFAULT, spif->base + SPIF_CSD);

	if (!of_property_read_u32(dev->of_node, "allwinner,tune-offset",
				  &spif->tune_offset)) {
		spif->tune_ref = devm_kmalloc(dev, SPIF_TUNE_SIZE, GFP_KERNEL);
		spif->tune_buf = devm_kmalloc(dev, SPIF_TUNE_SIZE, GFP_KERNEL);
		if (!spif->tune_ref || !spif->tune_buf)
			return -ENOMEM;
		spif->can_tune = true;
	}

	ctlr->mode_bits = SPI_CPOL | SPI_CPHA | SPI_RX_DUAL | SPI_TX_DUAL |
			  SPI_RX_QUAD | SPI_TX_QUAD;
	ctlr->num_chipselect = 4;
	ctlr->max_speed_hz = SPIF_MAX_SPEED;
	ctlr->mem_ops = &sunxi_spif_mem_ops;
	ctlr->mem_caps = &sunxi_spif_mem_caps;
	ctlr->dev.of_node = dev->of_node;

	return devm_spi_register_controller(dev, ctlr);
}

static const struct of_device_id sunxi_spif_match[] = {
	{ .compatible = "allwinner,sun252i-f101-spif" },
	{ }
};
MODULE_DEVICE_TABLE(of, sunxi_spif_match);

static struct platform_driver sunxi_spif_driver = {
	.probe = sunxi_spif_probe,
	.driver = {
		.name = "sunxi-spif",
		.of_match_table = sunxi_spif_match,
	},
};
module_platform_driver(sunxi_spif_driver);

MODULE_DESCRIPTION("Allwinner dedicated SPI flash controller");
MODULE_LICENSE("GPL");
