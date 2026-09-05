// SPDX-License-Identifier: GPL-2.0
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * Copyright (c) 2020 huangzhenwei@allwinnertech.com
 */

#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

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
#include "ccu_sdm.h"

#include "ccu-sun252i-f101.h"

#define SUNXI_CCU_VERSION               "0.0.5"

#define SUN252I_F101_PLL_PERI_CTRL_REG  0x0020
static struct ccu_nm pll_peri_clk = {
	.lock			= BIT(28),
	.enable			= BIT(31),
	.n				= _SUNXI_CCU_MULT_OFFSET_MIN_MAX(8, 8, 1, 53, 105),
	.min_rate		= 1272000000,
	.max_rate		= 2520000000,
	.common			= {
			.reg		= SUN252I_F101_PLL_PERI_CTRL_REG,
		.hw.init	= CLK_HW_INIT("pll-peri", "dcxo24M",
					&ccu_nm_ops,
					CLK_IS_CRITICAL),
	},
};

static SUNXI_CCU_M(pll_peri_2x_clk, "pll-peri-2x",
						"pll-peri", 0x0020,
						16, 3, CLK_IS_CRITICAL);

static CLK_FIXED_FACTOR(pll_peri_1x_clk, "pll-peri-1x",
						"pll-peri-2x", 2, 1, CLK_IS_CRITICAL);

static SUNXI_CCU_M(pll_peri_800m_clk, "pll-peri-800m",
						"pll-peri", 0x0020,
						20, 3, CLK_IS_CRITICAL);

static SUNXI_CCU_M(pll_peri_480m_clk, "pll-peri-480m",
						"pll-peri", 0x0020,
						2, 3, CLK_IS_CRITICAL);

#define SUN252I_F101_PLL_VIDEO0_CTRL_REG 0x0040
static struct ccu_nm pll_video0_4x_clk = {
	.lock			= BIT(28),
	.enable			= BIT(31),
	.n				= _SUNXI_CCU_MULT_OFFSET_MIN_MAX(8, 8, 1, 53, 105),
	.min_rate		= 1272000000,
	.max_rate		= 2520000000,
	.common			= {
			.reg		= SUN252I_F101_PLL_VIDEO0_CTRL_REG,
			.hw.init	= CLK_HW_INIT("pll-video0-4x", "dcxo24M",
								&ccu_nm_ops,
								0),
	},
};

static CLK_FIXED_FACTOR(pll_video0_2x_clk, "pll-video0-2x",
			"pll-video0-4x", 2, 1, 0);

static CLK_FIXED_FACTOR(pll_video0_1x_clk, "pll-video0-1x",
			"pll-video0-4x", 4, 1, 0);

#define SUN252I_F101_PLL_AUDIO1_CTRL_REG  0x0080
static struct ccu_nm pll_audio1_clk = {
	.lock			= BIT(28),
	.enable			= BIT(31),
	.n				= _SUNXI_CCU_MULT_MIN(8, 8, 12),
	.min_rate		= 1272000000,
	.max_rate		= 2520000000,
	.common			= {
			.reg		= SUN252I_F101_PLL_AUDIO1_CTRL_REG,
			.hw.init	= CLK_HW_INIT("pll-audio1", "dcxo24M",
								&ccu_nm_ops,
								0),
	},
};

static SUNXI_CCU_M(pll_audio1_div2_clk, "pll-audio1-div2",
						"pll-audio1", 0x0080,
						16, 3, CLK_IS_CRITICAL);


static SUNXI_CCU_M(pll_audio1_div5_clk, "pll-audio1-div5",
						"pll-audio1", 0x0080,
						20, 3, CLK_IS_CRITICAL);

static const char * const psi_parents[] = { "dcxo24M", "rtc32k", "clk1m-rc", "pll-peri-1x" };

static SUNXI_CCU_MP_WITH_MUX(psi_clk, "psi",
			psi_parents,
			0x0510,
			0, 2,
			8, 2,
			24, 2,
			0);

static const char * const apb0_parents[] = { "dcxo24M", "rtc32k", "psi", "pll-peri-1x" };

static SUNXI_CCU_MP_WITH_MUX(apb0_clk, "apb0",
			apb0_parents,
			0x0520,
			0, 5,
			8, 2,
			24, 2,
			0);

static const char * const apb1_parents[] = { "dcxo24M", "rtc32k", "psi", "pll-peri-1x" };

static SUNXI_CCU_MP_WITH_MUX(apb1_clk, "apb1",
			apb1_parents,
			0x0524,
			0, 5,
			8, 2,
			24, 2,
			0);

static const char * const apb_uart_parents[] = {
	"dcxo24M", "rtc32k", "pll-peri-480m", "pll-peri-1x",
	"pll-audio1-div2"
};

static SUNXI_CCU_M_WITH_MUX(apb_uart_clk, "apb-uart", apb_uart_parents,
			0x0528, 0, 5, 24, 3, 0);

static const char * const mbus_parents[] = {
	"dcxo24M", "pll-peri-2x", "pll-audio1-div2", "pll-ddr"
};

static SUNXI_CCU_M_WITH_MUX_GATE(mbus_clk, "mbus", mbus_parents,
			0x0544, 0, 5, 24, 2, BIT(31), CLK_IS_CRITICAL);

static const char * const de_parents[] = { "pll-peri-2x", "pll-video0-4x", "pll-audio1-div2" };

static SUNXI_CCU_M_WITH_MUX_GATE(de_clk, "de",
			de_parents, 0x0600,
			0, 5,
			24, 3,
			BIT(31),
			0);

static SUNXI_CCU_GATE(de_bus_clk, "de-bus",
			"dcxo24M",
			0x060C, BIT(0), 0);

static const char * const di_parents[] = { "pll-peri-2x", "pll-video0-4x", "pll-audio1-div2" };

static SUNXI_CCU_M_WITH_MUX_GATE(di_clk, "di",
			di_parents, 0x0620,
			0, 5,
			24, 3,
			BIT(31),
			0);

static SUNXI_CCU_GATE(di_bus_clk, "di-bus",
			"dcxo24M",
			0x062C, BIT(0), 0);

static const char * const g2d_parents[] = { "pll-peri-2x", "pll-video0-4x", "pll-audio1-div2" };

static SUNXI_CCU_M_WITH_MUX_GATE(g2d_clk, "g2d",
			g2d_parents, 0x0630,
			0, 5,
			24, 3,
			BIT(31),
			0);

static SUNXI_CCU_GATE(g2d_bus_clk, "g2d-bus",
			"dcxo24M",
			0x063C, BIT(0), 0);

static const char * const ce_parents[] = { "dcxo24M", "pll-peri-2x", "pll-peri-1x" };

static SUNXI_CCU_MP_WITH_MUX_GATE(ce_clk, "ce",
			ce_parents, 0x0680,
			0, 5,
			8, 2,
			24, 3,
			BIT(31),
			0);

static SUNXI_CCU_GATE(ce_bus_clk, "ce-bus",
			"dcxo24M",
			0x068C, BIT(0), 0);

static const char * const ve_parents[] = { "pll-peri-480m", "pll-peri-2x", "pll-audio1-div2", "pll-cpu", "pll-ddr" };

static SUNXI_CCU_M_WITH_MUX_GATE(ve_clk, "ve",
			ve_parents, 0x0690,
			0, 5,
			24, 3,
			BIT(31),
			0);

static SUNXI_CCU_GATE(ve_bus_clk, "ve-bus",
			"dcxo24M",
			0x069C, BIT(0), 0);

static SUNXI_CCU_GATE(dma_bus_clk, "dma-bus",
			"psi",
			0x070C, BIT(0), 0);

static SUNXI_CCU_GATE(hstimer_bus_clk, "hstimer-bus",
			"dcxo24M",
			0x073C, BIT(0), 0);

static SUNXI_CCU_GATE(avs_bus_clk, "avs-bus",
			"dcxo24M",
			0x0740, BIT(31), 0);

static SUNXI_CCU_GATE(dbgsys_bus_clk, "dbgsys-bus",
			"dcxo24M",
			0x078C, BIT(0), 0);

static const char * const pwm_bl_parents[] = { "pll-peri-2x", "pll-audio1-div2" };

static SUNXI_CCU_M_WITH_MUX_GATE(pwm_bl_clk, "pwm-bl",
			pwm_bl_parents, 0x0790,
			0, 5,
			24, 3,
			BIT(31),
			0);

static SUNXI_CCU_GATE(pwm_bl_bus_clk, "pwm-bl-bus",
			"dcxo24M",
			0x0794, BIT(0), 0);

static SUNXI_CCU_GATE(pwm_bus_clk, "pwm-bus",
			"dcxo24M",
			0x07AC, BIT(0), 0);

static SUNXI_CCU_GATE(g2d_m_bus_clk, "g2d-m-bus",
			"dcxo24M",
			0x0804, BIT(10), 0);

static SUNXI_CCU_GATE(csi_m_bus_clk, "csi-m-bus",
			"dcxo24M",
			0x0804, BIT(8), 0);

static SUNXI_CCU_GATE(tvin_m_bus_clk, "tvin-m-bus",
			"dcxo24M",
			0x0804, BIT(7), 0);

static SUNXI_CCU_GATE(ve_m_bus_clk, "ve-m-bus",
			"dcxo24M",
			0x0804, BIT(1), 0);

static SUNXI_CCU_GATE(dma_m_bus_clk, "dma-m-bus",
			"dcxo24M",
			0x0804, BIT(0), 0);

static SUNXI_CCU_GATE(dram_bus_clk, "dram-bus",
			"dcxo24M",
			0x080C, BIT(0), 0);

static const char * const smhc0_parents[] = { "dcxo24M", "pll-peri-1x", "pll-peri-2x", "pll-audio1-div2" };

static SUNXI_CCU_MP_WITH_MUX_GATE(smhc0_clk, "smhc0",
			smhc0_parents, 0x0830,
			0, 4,
			8, 2,
			24, 3,
			BIT(31),
			0);

static const char * const smhc2_parents[] = { "dcxo24M", "pll-peri-1x", "pll-peri-2x", "pll-peri-800m", "pll-audio1-div2" };

static SUNXI_CCU_MP_WITH_MUX_GATE(smhc2_clk, "smhc2",
			smhc2_parents, 0x0838,
			0, 4,
			8, 2,
			24, 3,
			BIT(31),
			0);

static SUNXI_CCU_GATE(smhc2_bus_clk, "smhc2-bus",
			"dcxo24M",
			0x084C, BIT(2), 0);

static SUNXI_CCU_GATE(smhc0_bus_clk, "smhc0-bus",
			"dcxo24M",
			0x084C, BIT(0), 0);

static SUNXI_CCU_GATE(uart5_bus_clk, "uart5-bus",
			"apb-uart",
			0x090C, BIT(5), 0);

static SUNXI_CCU_GATE(uart4_bus_clk, "uart4-bus",
			"apb-uart",
			0x090C, BIT(4), 0);

static SUNXI_CCU_GATE(uart3_bus_clk, "uart3-bus",
			"apb-uart",
			0x090C, BIT(3), 0);

static SUNXI_CCU_GATE(uart2_bus_clk, "uart2-bus",
			"apb-uart",
			0x090C, BIT(2), 0);

static SUNXI_CCU_GATE(uart1_bus_clk, "uart1-bus",
			"apb-uart",
			0x090C, BIT(1), 0);

static SUNXI_CCU_GATE(uart0_bus_clk, "uart0-bus",
			"apb-uart",
			0x090C, BIT(0), 0);

static SUNXI_CCU_GATE(twi2_bus_clk, "twi2-bus",
			"apb1",
			0x091C, BIT(2), 0);

static SUNXI_CCU_GATE(twi1_bus_clk, "twi1-bus",
			"apb1",
			0x091C, BIT(1), 0);

static SUNXI_CCU_GATE(twi0_bus_clk, "twi0-bus",
			"apb1",
			0x091C, BIT(0), 0);

static const char * const spi0_parents[] = { "dcxo24M", "pll-peri-1x", "pll-peri-2x", "pll-audio1-div2", "pll-audio1-div5" };

static SUNXI_CCU_MP_WITH_MUX_GATE(spi0_clk, "spi0",
			spi0_parents, 0x0940,
			0, 4,
			8, 2,
			24, 3,
			BIT(31),
			0);

static const char * const spi1_parents[] = { "dcxo24M", "pll-peri-1x", "pll-peri-2x", "pll-audio1-div2", "pll-audio1-div5" };

static SUNXI_CCU_MP_WITH_MUX_GATE(spi1_clk, "spi1",
			spi1_parents, 0x0944,
			0, 4,
			8, 2,
			24, 3,
			BIT(31),
			0);

static SUNXI_CCU_GATE(spi1_bus_clk, "spi1-bus",
			"dcxo24M",
			0x096C, BIT(1), 0);

static SUNXI_CCU_GATE(spi0_bus_clk, "spi0-bus",
			"dcxo24M",
			0x096C, BIT(0), 0);

static const char * const spif_parents[] = { "dcxo24M", "pll-peri-1x", "pll-peri-2x", "pll-peri-800m" };

static SUNXI_CCU_MP_WITH_MUX_GATE_NO_INDEX(spif_clk, "spif",
			spif_parents, 0x0980,
			0, 5,
			8, 5,
			24, 3,
			BIT(31), 0);

static SUNXI_CCU_GATE(spif_bus_clk, "spif-bus",
			"dcxo24M",
			0x0984, BIT(0), 0);

static const char * const irrx_parents[] = { "dcxo24M", "pll-peri-1x" };

static SUNXI_CCU_MP_WITH_MUX_GATE(irrx_clk, "irrx",
			irrx_parents, 0x09C0,
			0, 4,
			8, 2,
			24, 3,
			BIT(31),
			0);

static SUNXI_CCU_GATE(irrx_bus_clk, "irrx-bus",
			"dcxo24M",
			0x09CC, BIT(0), 0);

static SUNXI_CCU_GATE(can0_bus_clk, "can0-bus",
			"apb0",
			0x092C, BIT(0), 0);

static SUNXI_CCU_GATE(can1_bus_clk, "can1-bus",
			"apb0",
			0x092C, BIT(1), 0);

static SUNXI_CCU_GATE(gpadc_bus_clk, "gpadc-bus",
			"dcxo24M",
			0x09EC, BIT(0), 0);

static SUNXI_CCU_GATE(ths_bus_clk, "ths-bus",
			"dcxo24M",
			0x09FC, BIT(0), 0);

static const char * const i2s0_parents[] = { "pll-audio1-div2", "pll-audio1-div5", "pll-peri-1x" };

static SUNXI_CCU_MP_WITH_MUX_GATE(i2s0_clk, "i2s0",
			i2s0_parents, 0x0A10,
			0, 5,
			8, 2,
			24, 3,
			BIT(31),
			0);

static SUNXI_CCU_GATE(i2s0_bus_clk, "i2s0-bus",
			"dcxo24M",
			0x0A20, BIT(0), 0);

static const char * const owa_tx_parents[] = { "pll-audio1-div2", "pll-audio1-div5", "pll-peri-1x" };

static SUNXI_CCU_MP_WITH_MUX_GATE(owa_tx_clk, "owa-tx",
			owa_tx_parents, 0x0A24,
			0, 5,
			8, 2,
			24, 3,
			BIT(31),
			0);

static const char * const owa_rx_parents[] = { "pll-peri-1x", "pll-audio1-div2", "pll-audio1-div5" };

static SUNXI_CCU_MP_WITH_MUX_GATE(owa_rx_clk, "owa-rx",
			owa_rx_parents, 0x0A28,
			0, 5,
			8, 2,
			24, 3,
			BIT(31),
			0);

static SUNXI_CCU_GATE(owa_bus_clk, "owa-bus",
			"dcxo24M",
			0x0A2C, BIT(0), 0);

static const char * const audio_codec_dac_parents[] = { "pll-audio1-div2", "pll-audio1-div5", "pll-peri-1x" };

static SUNXI_CCU_MP_WITH_MUX_GATE(audio_codec_dac_clk, "audio-codec-dac",
			audio_codec_dac_parents, 0x0A50,
			0, 5,
			8, 2,
			24, 3,
			BIT(31),
			0);

static const char * const audio_codec_adc_parents[] = { "pll-audio1-div2", "pll-audio1-div5", "pll-peri-1x" };

static SUNXI_CCU_MP_WITH_MUX_GATE(audio_codec_adc_clk, "audio-codec-adc",
			audio_codec_adc_parents, 0x0A54,
			0, 5,
			8, 2,
			24, 3,
			BIT(31),
			0);

static SUNXI_CCU_GATE(audio_codec_bus_clk, "audio-codec-bus",
			"dcxo24M",
			0x0A5C, BIT(0), 0);

static SUNXI_CCU_GATE(usb_bus_clk, "usb-bus",
			"dcxo24M",
			0x0A70, BIT(31), 0);

static SUNXI_CCU_GATE(usbotg0_bus_clk, "usbotg0-bus",
			"dcxo24M",
			0x0A8C, BIT(8), 0);

static SUNXI_CCU_GATE(usbehci0_bus_clk, "usbehci0-bus",
			"dcxo24M",
			0x0A8C, BIT(4), 0);

static SUNXI_CCU_GATE(usbohci0_bus_clk, "usbohci0-bus",
			"dcxo24M",
			0x0A8C, BIT(0), 0);

static const char * const combophy0_parents[] = { "pll-video0-4x", "pll-peri-2x", "pll-audio1-div2" };

static SUNXI_CCU_M_WITH_MUX_GATE(combophy0_clk, "combophy0",
			combophy0_parents, 0x0AA0,
			0, 5,
			24, 3,
			BIT(31),
			0);

static SUNXI_CCU_GATE(combophy0_bus_clk, "combophy0-bus",
			"dcxo24M",
			0x0AA4, BIT(0), 0);

static SUNXI_CCU_GATE(dpss_top_bus_clk, "dpss-top-bus",
			"dcxo24M",
			0x0ABC, BIT(0), 0);

static const char * const dsi_parents[] = { "dcxo24M", "pll-peri-1x", "pll-video0-2x", "pll-audio1-div2" };

static SUNXI_CCU_M_WITH_MUX_GATE(dsi_clk, "dsi",
			dsi_parents, 0x0B24,
			0, 4,
			24, 3,
			BIT(31),
			0);

static SUNXI_CCU_GATE(dsi_bus_clk, "dsi-bus",
			"dcxo24M",
			0x0B4C, BIT(0), 0);

static const char * const tconlcd_parents[] = { "pll-video0-1x", "pll-video0-4x", "pll-peri-2x", "pll-audio1-div2" };

static SUNXI_CCU_MP_WITH_MUX_GATE(tconlcd_clk, "tconlcd",
			tconlcd_parents, 0x0B60,
			0, 4,
			8, 2,
			24, 3,
			BIT(31),
			0);

static SUNXI_CCU_GATE(tconlcd_bus_clk, "tconlcd-bus",
			"dcxo24M",
			0x0B7C, BIT(0), 0);

static const char * const csi_parents[] = { "pll-peri-2x", "pll-peri-800m", "pll-video0-2x", "pll-audio1-div2" };

static SUNXI_CCU_M_WITH_MUX_GATE(csi_clk, "csi",
			csi_parents, 0x0C04,
			0, 4,
			24, 3,
			BIT(31),
			0);

static const char * const csi_master_parents[] = { "dcxo24M", "pll-peri-1x", "pll-video0-4x", "pll-video0-2x", "pll-audio1-div2" };

static SUNXI_CCU_M_WITH_MUX_GATE(csi_master_clk, "csi-master",
			csi_master_parents, 0x0C08,
			0, 5,
			24, 3,
			BIT(31),
			0);

static SUNXI_CCU_GATE(csi_bus_clk, "csi-bus",
			"dcxo24M",
			0x0C1C, BIT(0), 0);

static const char * const tvin_master_parents[] = { "dcxo24M", "pll-video0-1x", "pll-peri-1x" };

static SUNXI_CCU_M_WITH_MUX_GATE(tvin_master_clk, "tvin-master",
			tvin_master_parents, 0x0C20,
			0, 5,
			24, 3,
			BIT(31),
			0);

static SUNXI_CCU_GATE(tvin_bus_clk, "tvin-bus",
			"dcxo24M",
			0x0C24, BIT(0), 0);

static SUNXI_CCU_GATE(tvin_top_bus_clk, "tvin-top-bus",
			"dcxo24M",
			0x0C28, BIT(0), 0);

static const char * const tpadc_parents[] = { "dcxo24M", "pll-audio1-div2" };

static SUNXI_CCU_MUX_WITH_GATE(tpadc_clk, "tpadc",
			tpadc_parents, 0x0C50,
			24, 3,
			BIT(31), 0);

static SUNXI_CCU_GATE(tpadc_bus_clk, "tpadc-bus",
			"dcxo24M",
			0x0C5C, BIT(0), 0);

static SUNXI_CCU_GATE(riscv_cfg_bus_clk, "riscv-cfg-bus",
			"dcxo24M",
			0x0D0C, BIT(0), 0);

static SUNXI_CCU_GATE(clk32k_bus_clk, "clk32k-bus",
			"dcxo24M",
			0x0F30, BIT(4), 0);

static SUNXI_CCU_GATE(clk25m_bus_clk, "clk25m-bus",
			"dcxo24M",
			0x0F30, BIT(3), 0);

static SUNXI_CCU_GATE(clk16m_bus_clk, "clk16m-bus",
			"dcxo24M",
			0x0F30, BIT(2), 0);

static SUNXI_CCU_GATE(clk12m_bus_clk, "clk12m-bus",
			"dcxo24M",
			0x0F30, BIT(1), 0);

static SUNXI_CCU_GATE(clk24m_bus_clk, "clk24m-bus",
			"dcxo24M",
			0x0F30, BIT(0), 0);

static SUNXI_CCU_M_WITH_GATE(pclk_clk, "pclk",
			"apb0", 0x0F38, 0,
			5, BIT(31), 0);

static SUNXI_CCU_GATE(clk_fanout2_bus_clk, "clk-fanout2-bus",
			"dcxo24M",
			0x0F3C, BIT(23), 0);

static SUNXI_CCU_GATE(clk_fanout1_bus_clk, "clk-fanout1-bus",
			"dcxo24M",
			0x0F3C, BIT(22), 0);

static SUNXI_CCU_GATE(clk_fanout0_bus_clk, "clk-fanout0-bus",
			"dcxo24M",
			0x0F3C, BIT(21), 0);
/* ccu_des_end */

/* rst_def_start */
static struct ccu_reset_map sun252i_f101_ccu_resets[] = {
	 [RST_BUS_PLL_SSC_RSTN]		= { 0x0200, BIT(30) },
	 [RST_MBUS]			= { 0x0540, BIT(30) },
	 [RST_BUS_DE]			= { 0x060c, BIT(16) },
	 [RST_BUS_DI]			= { 0x062c, BIT(16) },
	 [RST_BUS_G2D]			= { 0x063c, BIT(16) },
	 [RST_BUS_CE]			= { 0x068c, BIT(16) },
	 [RST_BUS_VE]			= { 0x069c, BIT(16) },
	 [RST_BUS_DMA]			= { 0x070c, BIT(16) },
	 [RST_BUS_HSTIME]		= { 0x073c, BIT(16) },
	 [RST_BUS_DBGSY]		= { 0x078c, BIT(16) },
	 [RST_BUS_PWM_BL]		= { 0x0794, BIT(16) },
	 [RST_BUS_PWM]			= { 0x07ac, BIT(16) },
	 [RST_BUS_DRAM_MODULE]		= { 0x0800, BIT(30) },
	 [RST_BUS_LPSRAM]		= { 0x0808, BIT(16) },
	 [RST_BUS_DRAM]			= { 0x080c, BIT(16) },
	 [RST_BUS_SMHC2]		= { 0x084c, BIT(18) },
	 [RST_BUS_SMHC0]		= { 0x084c, BIT(16) },
	 [RST_BUS_UART5]		= { 0x090c, BIT(21) },
	 [RST_BUS_UART4]		= { 0x090c, BIT(20) },
	 [RST_BUS_UART3]		= { 0x090c, BIT(19) },
	 [RST_BUS_UART2]		= { 0x090c, BIT(18) },
	 [RST_BUS_UART1]		= { 0x090c, BIT(17) },
	 [RST_BUS_UART0]		= { 0x090c, BIT(16) },
	 [RST_BUS_TWI2]			= { 0x091c, BIT(18) },
	 [RST_BUS_TWI1]			= { 0x091c, BIT(17) },
	 [RST_BUS_TWI0]			= { 0x091c, BIT(16) },
	 [RST_BUS_SPI1]			= { 0x096c, BIT(17) },
	 [RST_BUS_SPI0]			= { 0x096c, BIT(16) },
	 [RST_BUS_SPIF]			= { 0x0984, BIT(16) },
	 [RST_BUS_IRRX]			= { 0x09cc, BIT(16) },
	 [RST_BUS_CAN0]			= { 0x092c, BIT(16) },
	 [RST_BUS_CAN1]			= { 0x092c, BIT(17) },
	 [RST_BUS_GPADC]		= { 0x09ec, BIT(16) },
	 [RST_BUS_TH]			= { 0x09fc, BIT(16) },
	 [RST_BUS_I2S0]			= { 0x0a20, BIT(16) },
	 [RST_BUS_OWA]			= { 0x0a2c, BIT(16) },
	 [RST_BUS_AUDIO_CODEC]		= { 0x0a5c, BIT(16) },
	 [RST_USB_PHY0_RSTN]		= { 0x0a70, BIT(30) },
	 [RST_USB_OTG0]			= { 0x0a8c, BIT(24) },
	 [RST_USB_EHCI0]		= { 0x0a8c, BIT(20) },
	 [RST_USB_OHCI0]		= { 0x0a8c, BIT(16) },
	 [RST_BUS_DPSS_TOP]		= { 0x0abc, BIT(16) },
	 [RST_BUS_DSI]			= { 0x0b4c, BIT(16) },
	 [RST_BUS_TCONLCD]		= { 0x0b7c, BIT(16) },
	 [RST_BUS_LVDS0]		= { 0x0bac, BIT(16) },
	 [RST_BUS_CSI]			= { 0x0c1c, BIT(16) },
	 [RST_BUS_TVIN]			= { 0x0c24, BIT(16) },
	 [RST_BUS_TVIN_TOP]		= { 0x0c28, BIT(16) },
	 [RST_BUS_TPADC]		= { 0x0c5c, BIT(16) },
};
/* rst_def_end */

/* ccu_def_start */
static struct clk_hw_onecell_data sun252i_f101_hw_clks = {
	.hws	= {
		[CLK_PLL_PERI]			= &pll_peri_clk.common.hw,
		[CLK_PLL_PERI_2X]		= &pll_peri_2x_clk.common.hw,
		[CLK_PLL_PERI_1X]		= &pll_peri_1x_clk.hw,
		[CLK_PLL_PERI_800M]		= &pll_peri_800m_clk.common.hw,
		[CLK_PLL_PERI_480M]		= &pll_peri_480m_clk.common.hw,
		[CLK_PLL_VIDEO0_4X]		= &pll_video0_4x_clk.common.hw,
		[CLK_PLL_VIDEO0_2X]		= &pll_video0_2x_clk.hw,
		[CLK_PLL_VIDEO0_1X]		= &pll_video0_1x_clk.hw,
		[CLK_PLL_AUDIO1]		= &pll_audio1_clk.common.hw,
		[CLK_PLL_AUDIO1_DIV2]		= &pll_audio1_div2_clk.common.hw,
		[CLK_PLL_AUDIO1_DIV5]		= &pll_audio1_div5_clk.common.hw,
		[CLK_PSI]			= &psi_clk.common.hw,
		[CLK_APB0]			= &apb0_clk.common.hw,
		[CLK_APB1]			= &apb1_clk.common.hw,
		[CLK_APB_UART]			= &apb_uart_clk.common.hw,
		[CLK_MBUS]			= &mbus_clk.common.hw,
		[CLK_DE]			= &de_clk.common.hw,
		[CLK_BUS_DE]			= &de_bus_clk.common.hw,
		[CLK_DI]			= &di_clk.common.hw,
		[CLK_BUS_DI]			= &di_bus_clk.common.hw,
		[CLK_G2D]			= &g2d_clk.common.hw,
		[CLK_BUS_G2D]			= &g2d_bus_clk.common.hw,
		[CLK_CE]			= &ce_clk.common.hw,
		[CLK_BUS_CE]			= &ce_bus_clk.common.hw,
		[CLK_VE]			= &ve_clk.common.hw,
		[CLK_BUS_VE]			= &ve_bus_clk.common.hw,
		[CLK_BUS_DMA]			= &dma_bus_clk.common.hw,
		[CLK_BUS_HSTIMER]		= &hstimer_bus_clk.common.hw,
		[CLK_BUS_AVS]			= &avs_bus_clk.common.hw,
		[CLK_BUS_DBGSYS]		= &dbgsys_bus_clk.common.hw,
		[CLK_PWM_BL]			= &pwm_bl_clk.common.hw,
		[CLK_BUS_PWM_BL]		= &pwm_bl_bus_clk.common.hw,
		[CLK_BUS_PWM]			= &pwm_bus_clk.common.hw,
		[CLK_BUS_G2D_M]			= &g2d_m_bus_clk.common.hw,
		[CLK_BUS_CSI_M]			= &csi_m_bus_clk.common.hw,
		[CLK_BUS_TVIN_M]		= &tvin_m_bus_clk.common.hw,
		[CLK_BUS_VE_M]			= &ve_m_bus_clk.common.hw,
		[CLK_BUS_DMA_M]			= &dma_m_bus_clk.common.hw,
		[CLK_BUS_DRAM]			= &dram_bus_clk.common.hw,
		[CLK_SMHC0]			= &smhc0_clk.common.hw,
		[CLK_SMHC2]			= &smhc2_clk.common.hw,
		[CLK_BUS_SMHC2]			= &smhc2_bus_clk.common.hw,
		[CLK_BUS_SMHC0]			= &smhc0_bus_clk.common.hw,
		[CLK_BUS_UART5]			= &uart5_bus_clk.common.hw,
		[CLK_BUS_UART4]			= &uart4_bus_clk.common.hw,
		[CLK_BUS_UART3]			= &uart3_bus_clk.common.hw,
		[CLK_BUS_UART2]			= &uart2_bus_clk.common.hw,
		[CLK_BUS_UART1]			= &uart1_bus_clk.common.hw,
		[CLK_BUS_UART0]			= &uart0_bus_clk.common.hw,
		[CLK_BUS_TWI2]			= &twi2_bus_clk.common.hw,
		[CLK_BUS_TWI1]			= &twi1_bus_clk.common.hw,
		[CLK_BUS_TWI0]			= &twi0_bus_clk.common.hw,
		[CLK_SPI0]			= &spi0_clk.common.hw,
		[CLK_SPI1]			= &spi1_clk.common.hw,
		[CLK_BUS_SPI1]			= &spi1_bus_clk.common.hw,
		[CLK_BUS_SPI0]			= &spi0_bus_clk.common.hw,
		[CLK_SPIF]			= &spif_clk.common.hw,
		[CLK_BUS_SPIF]			= &spif_bus_clk.common.hw,
		[CLK_IRRX]			= &irrx_clk.common.hw,
		[CLK_BUS_IRRX]			= &irrx_bus_clk.common.hw,
		[CLK_CAN0]			= &can0_bus_clk.common.hw,
		[CLK_CAN1]			= &can1_bus_clk.common.hw,
		[CLK_BUS_GPADC]			= &gpadc_bus_clk.common.hw,
		[CLK_BUS_THS]			= &ths_bus_clk.common.hw,
		[CLK_I2S0]			= &i2s0_clk.common.hw,
		[CLK_BUS_I2S0]			= &i2s0_bus_clk.common.hw,
		[CLK_OWA_TX]			= &owa_tx_clk.common.hw,
		[CLK_OWA_RX]			= &owa_rx_clk.common.hw,
		[CLK_BUS_OWA]			= &owa_bus_clk.common.hw,
		[CLK_AUDIO_CODEC_DAC]		= &audio_codec_dac_clk.common.hw,
		[CLK_AUDIO_CODEC_ADC]		= &audio_codec_adc_clk.common.hw,
		[CLK_BUS_AUDIO_CODEC]		= &audio_codec_bus_clk.common.hw,
		[CLK_BUS_USB]			= &usb_bus_clk.common.hw,
		[CLK_BUS_USBOTG0]		= &usbotg0_bus_clk.common.hw,
		[CLK_BUS_USBEHCI0]		= &usbehci0_bus_clk.common.hw,
		[CLK_BUS_USBOHCI0]		= &usbohci0_bus_clk.common.hw,
		[CLK_COMBOPHY0]			= &combophy0_clk.common.hw,
		[CLK_BUS_COMBOPHY0]		= &combophy0_bus_clk.common.hw,
		[CLK_BUS_DPSS_TOP]		= &dpss_top_bus_clk.common.hw,
		[CLK_DSI]			= &dsi_clk.common.hw,
		[CLK_BUS_DSI]			= &dsi_bus_clk.common.hw,
		[CLK_TCONLCD]			= &tconlcd_clk.common.hw,
		[CLK_BUS_TCONLCD]		= &tconlcd_bus_clk.common.hw,
		[CLK_CSI]			= &csi_clk.common.hw,
		[CLK_CSI_MASTER]		= &csi_master_clk.common.hw,
		[CLK_BUS_CSI]			= &csi_bus_clk.common.hw,
		[CLK_TVIN_MASTER]		= &tvin_master_clk.common.hw,
		[CLK_BUS_TVIN]			= &tvin_bus_clk.common.hw,
		[CLK_BUS_TVIN_TOP]		= &tvin_top_bus_clk.common.hw,
		[CLK_TPADC]			= &tpadc_clk.common.hw,
		[CLK_BUS_TPADC]			= &tpadc_bus_clk.common.hw,
		[CLK_BUS_RISCV_CFG]		= &riscv_cfg_bus_clk.common.hw,
		[CLK_BUS_CLK32K]		= &clk32k_bus_clk.common.hw,
		[CLK_BUS_CLK25M]		= &clk25m_bus_clk.common.hw,
		[CLK_BUS_CLK16M]		= &clk16m_bus_clk.common.hw,
		[CLK_BUS_CLK12M]		= &clk12m_bus_clk.common.hw,
		[CLK_BUS_CLK24M]		= &clk24m_bus_clk.common.hw,
		[CLK_PCLK]			= &pclk_clk.common.hw,
		[CLK_BUS_CLK_FANOUT2]		= &clk_fanout2_bus_clk.common.hw,
		[CLK_BUS_CLK_FANOUT1]		= &clk_fanout1_bus_clk.common.hw,
		[CLK_BUS_CLK_FANOUT0]		= &clk_fanout0_bus_clk.common.hw,
	},
	.num	= CLK_NUMBER,
};
/* ccu_def_end */

static struct ccu_common *sun252i_f101_ccu_clks[] = {
	&pll_peri_clk.common,
	&pll_peri_2x_clk.common,
	&pll_peri_800m_clk.common,
	&pll_peri_480m_clk.common,
	&pll_video0_4x_clk.common,
	&pll_audio1_clk.common,
	&pll_audio1_div2_clk.common,
	&pll_audio1_div5_clk.common,
	&psi_clk.common,
	&apb0_clk.common,
	&apb1_clk.common,
	&apb_uart_clk.common,
	&mbus_clk.common,
	&de_clk.common,
	&de_bus_clk.common,
	&di_clk.common,
	&di_bus_clk.common,
	&g2d_clk.common,
	&g2d_bus_clk.common,
	&ce_clk.common,
	&ce_bus_clk.common,
	&ve_clk.common,
	&ve_bus_clk.common,
	&dma_bus_clk.common,
	&hstimer_bus_clk.common,
	&avs_bus_clk.common,
	&dbgsys_bus_clk.common,
	&pwm_bl_clk.common,
	&pwm_bl_bus_clk.common,
	&pwm_bus_clk.common,
	&g2d_m_bus_clk.common,
	&csi_m_bus_clk.common,
	&tvin_m_bus_clk.common,
	&ve_m_bus_clk.common,
	&dma_m_bus_clk.common,
	&dram_bus_clk.common,
	&smhc0_clk.common,
	&smhc2_clk.common,
	&smhc2_bus_clk.common,
	&smhc0_bus_clk.common,
	&uart5_bus_clk.common,
	&uart4_bus_clk.common,
	&uart3_bus_clk.common,
	&uart2_bus_clk.common,
	&uart1_bus_clk.common,
	&uart0_bus_clk.common,
	&twi2_bus_clk.common,
	&twi1_bus_clk.common,
	&twi0_bus_clk.common,
	&spi0_clk.common,
	&spi1_clk.common,
	&spi1_bus_clk.common,
	&spi0_bus_clk.common,
	&spif_clk.common,
	&spif_bus_clk.common,
	&irrx_clk.common,
	&irrx_bus_clk.common,
	&can0_bus_clk.common,
	&can1_bus_clk.common,
	&gpadc_bus_clk.common,
	&ths_bus_clk.common,
	&i2s0_clk.common,
	&i2s0_bus_clk.common,
	&owa_tx_clk.common,
	&owa_rx_clk.common,
	&owa_bus_clk.common,
	&audio_codec_dac_clk.common,
	&audio_codec_adc_clk.common,
	&audio_codec_bus_clk.common,
	&usb_bus_clk.common,
	&usbotg0_bus_clk.common,
	&usbehci0_bus_clk.common,
	&usbohci0_bus_clk.common,
	&combophy0_clk.common,
	&combophy0_bus_clk.common,
	&dpss_top_bus_clk.common,
	&dsi_clk.common,
	&dsi_bus_clk.common,
	&tconlcd_clk.common,
	&tconlcd_bus_clk.common,
	&csi_clk.common,
	&csi_master_clk.common,
	&csi_bus_clk.common,
	&tvin_master_clk.common,
	&tvin_bus_clk.common,
	&tvin_top_bus_clk.common,
	&tpadc_clk.common,
	&tpadc_bus_clk.common,
	&riscv_cfg_bus_clk.common,
	&clk32k_bus_clk.common,
	&clk25m_bus_clk.common,
	&clk16m_bus_clk.common,
	&clk12m_bus_clk.common,
	&clk24m_bus_clk.common,
	&pclk_clk.common,
	&clk_fanout2_bus_clk.common,
	&clk_fanout1_bus_clk.common,
	&clk_fanout0_bus_clk.common,
};


static const struct sunxi_ccu_desc sun252i_f101_ccu_desc = {
	.ccu_clks	= sun252i_f101_ccu_clks,
	.num_ccu_clks	= ARRAY_SIZE(sun252i_f101_ccu_clks),

	.hw_clks	= &sun252i_f101_hw_clks,

	.resets		= sun252i_f101_ccu_resets,
	.num_resets	= ARRAY_SIZE(sun252i_f101_ccu_resets),
};

static int sun252i_f101_ccu_probe(struct platform_device *pdev)
{
	void __iomem *reg;

	reg = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(reg))
		return PTR_ERR(reg);

	return devm_sunxi_ccu_probe(&pdev->dev, reg, &sun252i_f101_ccu_desc);
}

static const struct of_device_id sun252i_f101_ccu_ids[] = {
	{ .compatible = "allwinner,sun252i-f101-ccu" },
	{ }
};
MODULE_DEVICE_TABLE(of, sun252i_f101_ccu_ids);

static struct platform_driver sun252i_f101_ccu_driver = {
	.probe	= sun252i_f101_ccu_probe,
	.driver	= {
		.name	= "sun252i-f101-ccu",
		.of_match_table	= sun252i_f101_ccu_ids,
	},
};
module_platform_driver(sun252i_f101_ccu_driver);

MODULE_IMPORT_NS("SUNXI_CCU");
MODULE_DESCRIPTION("Allwinner sun252i F101 clock controller");
MODULE_AUTHOR("wangxinfeng<wangxinfeng@allwinnertech.com>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(SUNXI_CCU_VERSION);
