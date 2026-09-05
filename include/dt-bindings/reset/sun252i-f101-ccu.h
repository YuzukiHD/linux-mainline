/* SPDX-License-Identifier: (GPL-2.0-only OR MIT) */
/* Reset bindings for the Allwinner sun252i F101 CCU. */

#ifndef _DT_BINDINGS_RESET_SUN252I_F101_CCU_H_
#define _DT_BINDINGS_RESET_SUN252I_F101_CCU_H_

#define RST_BUS_PLL_SSC_RSTN		0
#define RST_MBUS			1
#define RST_BUS_DE			2
#define RST_BUS_DI			3
#define RST_BUS_G2D			4
#define RST_BUS_CE			5
#define RST_BUS_VE			6
#define RST_BUS_DMA			7
#define RST_BUS_HSTIME			8
#define RST_BUS_DBGSY			9
#define RST_BUS_PWM_BL		10
#define RST_BUS_PWM			11
#define RST_BUS_DRAM_MODULE		12
#define RST_BUS_LPSRAM			13
#define RST_BUS_DRAM			14
#define RST_BUS_SMHC2			15
#define RST_BUS_SMHC0			16
#define RST_BUS_UART5			17
#define RST_BUS_UART4			18
#define RST_BUS_UART3			19
#define RST_BUS_UART2			20
#define RST_BUS_UART1			21
#define RST_BUS_UART0			22
#define RST_BUS_TWI2			23
#define RST_BUS_TWI1			24
#define RST_BUS_TWI0			25
#define RST_BUS_SPI1			26
#define RST_BUS_SPI0			27
#define RST_BUS_SPIF			28
#define RST_BUS_IRRX			29
#define RST_BUS_CAN0			30
#define RST_BUS_CAN1			31
#define RST_BUS_GPADC			32
#define RST_BUS_TH			33
#define RST_BUS_I2S0			34
#define RST_BUS_OWA			35
#define RST_BUS_AUDIO_CODEC		36
#define RST_USB_PHY0_RSTN		37
#define RST_USB_OTG0			38
#define RST_USB_EHCI0			39
#define RST_USB_OHCI0			40
#define RST_BUS_DPSS_TOP		41
#define RST_BUS_DSI			42
#define RST_BUS_TCONLCD		43
#define RST_BUS_LVDS0			44
#define RST_BUS_CSI			45
#define RST_BUS_TVIN			46
#define RST_BUS_TVIN_TOP		47
#define RST_BUS_TPADC			48

#define RST_NUMBER			49

#define SUN252I_F101_RST_BUS_UART0	RST_BUS_UART0
#define SUN252I_F101_RST_BUS_UART1	RST_BUS_UART1
#define SUN252I_F101_RST_BUS_UART2	RST_BUS_UART2
#define SUN252I_F101_RST_BUS_UART3	RST_BUS_UART3
#define SUN252I_F101_RST_BUS_UART4	RST_BUS_UART4
#define SUN252I_F101_RST_BUS_UART5	RST_BUS_UART5
#define SUN252I_F101_RST_NUMBER		RST_NUMBER

#endif
