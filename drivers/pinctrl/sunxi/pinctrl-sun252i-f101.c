// SPDX-License-Identifier: GPL-2.0-only
/*
 * Allwinner sun252i F101 pinctrl driver.
 *
 * Copyright (c) 2023 Allwinner Technology Co., Ltd.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/platform_device.h>

#include "pinctrl-sunxi.h"

/* Pin banks are: A B C D E F. */
static const struct sunxi_desc_pin sun252i_f101_pins[] = {

	/* bank A */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(A, 0),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "tp"),            /* tp */
		SUNXI_FUNCTION(0x3, "pwm0_0"),        /* pwm_1 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 0, 0),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(A, 1),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "tp"),            /* tp */
		SUNXI_FUNCTION(0x3, "pwm0_1"),        /* pwm0_1 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 0, 1),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(A, 2),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "tp"),            /* tp */
		SUNXI_FUNCTION(0x3, "pwm0_2"),        /* pwm0_2 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 0, 2),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(A, 3),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "tp"),            /* tp */
		SUNXI_FUNCTION(0x3, "pwm0_3"),        /* pwm0_3 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 0, 3),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	/* bank B */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(B, 0),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "boost1"),        /* boost1 */
		SUNXI_FUNCTION(0x3, "twi1"),          /* twi1 */
		SUNXI_FUNCTION(0x4, "uart1"),         /* uart1 */
		SUNXI_FUNCTION(0x5, "uart2"),         /* uart2 */
		SUNXI_FUNCTION(0x6, "can"),           /* can */
		SUNXI_FUNCTION(0x7, "pwm0_0"),        /* pwm0_0 */
		SUNXI_FUNCTION(0x8, "spif0"),         /* spif0 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 1, 0),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(B, 1),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "boost1"),        /* boost1 */
		SUNXI_FUNCTION(0x3, "twi1"),          /* twi1 */
		SUNXI_FUNCTION(0x4, "uart1"),         /* uart1 */
		SUNXI_FUNCTION(0x5, "uart2"),         /* uart2 */
		SUNXI_FUNCTION(0x6, "can"),           /* can */
		SUNXI_FUNCTION(0x7, "pwm0_1"),        /* pwm0_1 */
		SUNXI_FUNCTION(0x8, "ir"),            /* ir */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 1, 1),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(B, 2),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "boost0"),        /* boost0 */
		SUNXI_FUNCTION(0x3, "twi2"),          /* twi2 */
		SUNXI_FUNCTION(0x4, "uart1"),         /* uart1 */
		SUNXI_FUNCTION(0x5, "uart2"),         /* uart2 */
		SUNXI_FUNCTION(0x6, "can"),           /* can */
		SUNXI_FUNCTION(0x7, "pwm0_2"),        /* pwm0_2 */
		SUNXI_FUNCTION(0x8, "spi0"),          /* spi0 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 1, 2),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(B, 3),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "boost0"),        /* boost0 */
		SUNXI_FUNCTION(0x3, "twi2"),          /* twi2 */
		SUNXI_FUNCTION(0x4, "uart1"),         /* uart1 */
		SUNXI_FUNCTION(0x5, "uart2"),         /* uart2 */
		SUNXI_FUNCTION(0x6, "can"),           /* can */
		SUNXI_FUNCTION(0x7, "pwm0_3"),        /* pwm0_3 */
		SUNXI_FUNCTION(0x8, "ir"),            /* ir */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 1, 3),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	/* bank C */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(C, 0),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "sdc2"),          /* sdc2 */
		SUNXI_FUNCTION(0x3, "spi0"),          /* spi0 */
		SUNXI_FUNCTION(0x4, "spif0"),         /* spif0 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 2, 0),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(C, 1),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "sdc2"),          /* sdc2 */
		SUNXI_FUNCTION(0x3, "spi0"),          /* spi0 */
		SUNXI_FUNCTION(0x4, "spif0"),         /* spif0 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 2, 1),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(C, 2),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "sdc2"),          /* sdc2 */
		SUNXI_FUNCTION(0x3, "spi0"),          /* spi0 */
		SUNXI_FUNCTION(0x4, "spif0"),         /* spif0 */
		SUNXI_FUNCTION(0x5, "boot"),          /* boot */
		SUNXI_FUNCTION(0x6, "uart3"),         /* uart3 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 2, 2),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(C, 3),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "sdc2"),          /* sdc2 */
		SUNXI_FUNCTION(0x3, "spi0"),          /* spi0 */
		SUNXI_FUNCTION(0x4, "spif0"),         /* spif0 */
		SUNXI_FUNCTION(0x5, "twi1"),          /* twi1 */
		SUNXI_FUNCTION(0x6, "uart3"),         /* uart3 */
		SUNXI_FUNCTION(0x7, "pwm0_0"),        /* pwm0_0 */
		SUNXI_FUNCTION(0x8, "dbg"),           /* dbg */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 2, 3),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(C, 4),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "sdc2"),          /* sdc2 */
		SUNXI_FUNCTION(0x3, "spi0"),          /* spi0 */
		SUNXI_FUNCTION(0x4, "spif0"),         /* spif0 */
		SUNXI_FUNCTION(0x5, "boot"),          /* boot */
		SUNXI_FUNCTION(0x6, "uart3"),         /* uart3 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 2, 4),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(C, 5),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "sdc2"),          /* sdc2 */
		SUNXI_FUNCTION(0x3, "spi0"),          /* spi0 */
		SUNXI_FUNCTION(0x4, "spif0"),         /* spif0 */
		SUNXI_FUNCTION(0x5, "twi1"),          /* twi1 */
		SUNXI_FUNCTION(0x6, "uart3"),         /* uart3 */
		SUNXI_FUNCTION(0x7, "pwm0_1"),        /* pwm0_1 */
		SUNXI_FUNCTION(0x8, "tcon"),          /* tcon */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 2, 5),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	/* bank D */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(D, 0),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "lcd0"),          /* lcd0 */
		SUNXI_FUNCTION(0x3, "spi1"),          /* spi1 */
		SUNXI_FUNCTION(0x4, "twi1"),          /* twi1 */
		SUNXI_FUNCTION(0x5, "uart3"),         /* uart3 */
		SUNXI_FUNCTION(0x6, "twi0"),          /* twi0 */
		SUNXI_FUNCTION(0x7, "rjtag"),         /* rjtag */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 3, 0),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(D, 1),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "lcd0"),          /* lcd0 */
		SUNXI_FUNCTION(0x3, "spi1"),          /* spi1 */
		SUNXI_FUNCTION(0x4, "twi1"),          /* twi1 */
		SUNXI_FUNCTION(0x5, "uart3"),         /* uart3 */
		SUNXI_FUNCTION(0x6, "pwm0_0"),        /* pwm0_0 */
		SUNXI_FUNCTION(0x7, "rjtag"),         /* rjtag */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 3, 1),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(D, 2),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "lcd0"),          /* lcd0 */
		SUNXI_FUNCTION(0x3, "spi1"),          /* spi1 */
		SUNXI_FUNCTION(0x4, "uart2"),         /* uart2 */
		SUNXI_FUNCTION(0x5, "twi0"),          /* twi0 */
		SUNXI_FUNCTION(0x6, "uart1"),         /* uart1 */
		SUNXI_FUNCTION(0x7, "rjtag"),         /* rjtag */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 3, 2),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(D, 3),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "lcd0"),          /* lcd0 */
		SUNXI_FUNCTION(0x3, "spi1"),          /* spi1 */
		SUNXI_FUNCTION(0x4, "uart2"),         /* uart2 */
		SUNXI_FUNCTION(0x5, "uart3"),         /* uart3 */
		SUNXI_FUNCTION(0x6, "uart1"),         /* uart1 */
		SUNXI_FUNCTION(0x7, "rjtag"),         /* rjtag */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 3, 3),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(D, 4),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "lcd0"),          /* lcd0 */
		SUNXI_FUNCTION(0x3, "spi1"),          /* spi1 */
		SUNXI_FUNCTION(0x4, "uart2"),         /* uart2 */
		SUNXI_FUNCTION(0x5, "uart3"),         /* uart3 */
		SUNXI_FUNCTION(0x6, "twi0"),          /* twi0 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 3, 4),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(D, 5),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "lcd0"),          /* lcd0 */
		SUNXI_FUNCTION(0x3, "spi1"),          /* spi1 */
		SUNXI_FUNCTION(0x4, "uart2"),         /* uart2 */
		SUNXI_FUNCTION(0x5, "ir"),            /* ir */
		SUNXI_FUNCTION(0x6, "twi0"),          /* twi0 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 3, 5),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(D, 6),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "lcd0"),          /* lcd0 */
		SUNXI_FUNCTION(0x3, "spi1"),          /* spi1 */
		SUNXI_FUNCTION(0x4, "i2s0_mclk"),     /* i2s0_mclk */
		SUNXI_FUNCTION(0x5, "uart4"),         /* uart4 */
		SUNXI_FUNCTION(0x6, "pwm0_0"),        /* pwm0_0 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 3, 6),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(D, 7),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "lcd0"),          /* lcd0 */
		SUNXI_FUNCTION(0x3, "ir"),            /* ir */
		SUNXI_FUNCTION(0x4, "i2s0_bclk"),     /* i2s0_bclk */
		SUNXI_FUNCTION(0x5, "uart4"),         /* uart4 */
		SUNXI_FUNCTION(0x6, "pwm0_1"),        /* pwm0_1 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 3, 7),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(D, 8),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "lcd0"),          /* lcd0 */
		SUNXI_FUNCTION(0x3, "twi1"),          /* twi1 */
		SUNXI_FUNCTION(0x4, "i2s0_lrck"),     /* i2s0_lrck */
		SUNXI_FUNCTION(0x5, "uart4"),         /* uart4 */
		SUNXI_FUNCTION(0x6, "pwm0_2"),        /* pwm0_2 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 3, 8),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(D, 9),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "lcd0"),          /* lcd0 */
		SUNXI_FUNCTION(0x3, "twi1"),          /* twi1 */
		SUNXI_FUNCTION(0x4, "i2s0_dout0"),    /* i2s0_dout0 */
		SUNXI_FUNCTION(0x5, "uart4"),         /* uart4 */
		SUNXI_FUNCTION(0x6, "pwm0_3"),        /* pwm0_3 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 3, 9),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(D, 10),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "lcd0"),          /* lcd0 */
		SUNXI_FUNCTION(0x3, "twi2"),          /* twi2 */
		SUNXI_FUNCTION(0x4, "i2s0_din0"),     /* i2s0_din0 */
		SUNXI_FUNCTION(0x5, "uart1"),         /* uart1 */
		SUNXI_FUNCTION(0x6, "uart5"),         /* uart5 */
		SUNXI_FUNCTION(0x7, "pwm0_3"),        /* pwm0_3 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 3, 10),  /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(D, 11),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "lcd0"),          /* lcd0 */
		SUNXI_FUNCTION(0x3, "twi2"),          /* twi2 */
		SUNXI_FUNCTION(0x4, "owa"),           /* owa */
		SUNXI_FUNCTION(0x5, "uart1"),         /* uart1 */
		SUNXI_FUNCTION(0x6, "uart5"),         /* uart5 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 3, 11),  /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(D, 12),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "lcd0"),          /* lcd0 */
		SUNXI_FUNCTION(0x3, "lvds0"),         /* lvds0 */
		SUNXI_FUNCTION(0x4, "dsi"),           /* dsi */
		SUNXI_FUNCTION(0x5, "uart3"),         /* uart3 */
		SUNXI_FUNCTION(0x6, "twi0"),          /* twi0 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 3, 12),  /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(D, 13),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "lcd0"),          /* lcd0 */
		SUNXI_FUNCTION(0x3, "lvds0"),         /* lvds0 */
		SUNXI_FUNCTION(0x4, "dsi"),           /* dsi */
		SUNXI_FUNCTION(0x5, "uart2"),         /* uart2 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 3, 13),  /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(D, 14),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "lcd0"),          /* lcd0 */
		SUNXI_FUNCTION(0x3, "lvds0"),         /* lvds0 */
		SUNXI_FUNCTION(0x4, "dsi"),           /* dsi */
		SUNXI_FUNCTION(0x5, "uart2"),         /* uart2 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 3, 14),  /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(D, 15),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "lcd0"),          /* lcd0 */
		SUNXI_FUNCTION(0x3, "lvds0"),         /* lvds0 */
		SUNXI_FUNCTION(0x4, "dsi"),           /* dsi */
		SUNXI_FUNCTION(0x5, "uart2"),         /* uart2 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 3, 15),  /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(D, 16),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "lcd0"),          /* lcd0 */
		SUNXI_FUNCTION(0x3, "lvds0"),         /* lvds0 */
		SUNXI_FUNCTION(0x4, "dsi"),           /* dsi */
		SUNXI_FUNCTION(0x5, "uart2"),         /* uart2 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 3, 16),  /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(D, 17),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "lcd0"),          /* lcd0 */
		SUNXI_FUNCTION(0x3, "lvds0"),         /* lvds0 */
		SUNXI_FUNCTION(0x4, "dsi"),           /* dsi */
		SUNXI_FUNCTION(0x5, "uart5"),         /* uart5 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 3, 17),  /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(D, 18),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "lcd0"),          /* lcd0 */
		SUNXI_FUNCTION(0x3, "lvds0"),         /* lvds0 */
		SUNXI_FUNCTION(0x4, "dsi"),           /* dsi */
		SUNXI_FUNCTION(0x5, "uart5"),         /* uart5 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 3, 18),  /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(D, 19),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "lcd0"),          /* lcd0 */
		SUNXI_FUNCTION(0x3, "lvds0"),         /* lvds0 */
		SUNXI_FUNCTION(0x4, "dsi"),           /* dsi */
		SUNXI_FUNCTION(0x5, "uart5"),         /* uart5 */
		SUNXI_FUNCTION(0x6, "uart4"),         /* uart4 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 3, 19),  /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(D, 20),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "lcd0"),          /* lcd0 */
		SUNXI_FUNCTION(0x3, "lvds0"),         /* lvds0 */
		SUNXI_FUNCTION(0x4, "dsi"),           /* dsi */
		SUNXI_FUNCTION(0x5, "uart5"),         /* uart5 */
		SUNXI_FUNCTION(0x6, "uart4"),         /* uart4 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 3, 20),  /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(D, 21),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "lcd0"),          /* lcd0 */
		SUNXI_FUNCTION(0x3, "lvds0"),         /* lvds0 */
		SUNXI_FUNCTION(0x4, "dsi"),           /* dsi */
		SUNXI_FUNCTION(0x5, "uart1"),         /* uart1 */
		SUNXI_FUNCTION(0x6, "uart4"),         /* uart4 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 3, 21),  /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(D, 22),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "owa"),           /* owa */
		SUNXI_FUNCTION(0x3, "ir"),            /* ir */
		SUNXI_FUNCTION(0x4, "pwm0_2"),        /* pwm0_2 */
		SUNXI_FUNCTION(0x5, "uart1"),         /* uart1 */
		SUNXI_FUNCTION(0x6, "uart4"),         /* uart4 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 3, 22),  /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	/* bank E */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(E, 0),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "ncsi0"),         /* ncsi0 */
		SUNXI_FUNCTION(0x3, "lcd0"),          /* lcd0 */
		SUNXI_FUNCTION(0x4, "twi1"),          /* twi1 */
		SUNXI_FUNCTION(0x5, "i2s0_mclk"),     /* i2s0_mclk */
		SUNXI_FUNCTION(0x6, "uart0"),         /* uart0 */
		SUNXI_FUNCTION(0x7, "can"),           /* can */
		SUNXI_FUNCTION(0x8, "spi1"),          /* spi1 */
		SUNXI_FUNCTION(0x9, "sdc2"),          /* sdc2 */
		SUNXI_FUNCTION(0xa, "clk"),           /* clk */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 4, 0),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(E, 1),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "ncsi0"),         /* ncsi0 */
		SUNXI_FUNCTION(0x3, "lcd0"),          /* lcd0 */
		SUNXI_FUNCTION(0x4, "twi1"),          /* twi1 */
		SUNXI_FUNCTION(0x5, "i2s0_bclk"),     /* i2s0_bclk */
		SUNXI_FUNCTION(0x6, "uart0"),         /* uart0 */
		SUNXI_FUNCTION(0x7, "can"),           /* can */
		SUNXI_FUNCTION(0x8, "spi1"),          /* spi1 */
		SUNXI_FUNCTION(0x9, "sdc2"),          /* sdc2 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 4, 1),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(E, 2),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "ncsi0"),         /* ncsi0 */
		SUNXI_FUNCTION(0x3, "lcd0"),          /* lcd0 */
		SUNXI_FUNCTION(0x4, "twi0"),          /* twi0 */
		SUNXI_FUNCTION(0x5, "i2s0_lrck"),     /* i2s0_lrck */
		SUNXI_FUNCTION(0x6, "uart4"),         /* uart4 */
		SUNXI_FUNCTION(0x7, "can"),           /* can */
		SUNXI_FUNCTION(0x8, "spi1"),          /* spi1 */
		SUNXI_FUNCTION(0x9, "sdc2"),          /* sdc2 */
		SUNXI_FUNCTION(0xa, "pwm0_0"),        /* pwm0_0 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 4, 2),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(E, 3),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "ncsi0"),         /* ncsi0 */
		SUNXI_FUNCTION(0x3, "lcd0"),          /* lcd0 */
		SUNXI_FUNCTION(0x4, "twi0"),          /* twi0 */
		SUNXI_FUNCTION(0x5, "i2s0_dout0"),    /* i2s0_dout0 */
		SUNXI_FUNCTION(0x6, "uart4"),         /* uart4 */
		SUNXI_FUNCTION(0x7, "can"),           /* can */
		SUNXI_FUNCTION(0x8, "spi1"),          /* spi1 */
		SUNXI_FUNCTION(0x9, "sdc2"),          /* sdc2 */
		SUNXI_FUNCTION(0xa, "pwm0_1"),        /* pwm0_1 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 4, 3),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(E, 4),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "ncsi0"),         /* ncsi0 */
		SUNXI_FUNCTION(0x3, "lcd0"),          /* lcd0 */
		SUNXI_FUNCTION(0x4, "twi2"),          /* twi2 */
		SUNXI_FUNCTION(0x5, "i2s0_din0"),     /* i2s0_din0 */
		SUNXI_FUNCTION(0x6, "uart4"),         /* uart4 */
		SUNXI_FUNCTION(0x7, "uart5"),         /* uart5 */
		SUNXI_FUNCTION(0x8, "spi1"),          /* spi1 */
		SUNXI_FUNCTION(0x9, "sdc2"),          /* sdc2 */
		SUNXI_FUNCTION(0xa, "pwm0_2"),        /* pwm0_2 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 4, 4),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(E, 5),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "ncsi0"),         /* ncsi0 */
		SUNXI_FUNCTION(0x3, "lcd0"),          /* lcd0 */
		SUNXI_FUNCTION(0x4, "twi2"),          /* twi2 */
		SUNXI_FUNCTION(0x5, "ir"),            /* ir */
		SUNXI_FUNCTION(0x6, "uart4"),         /* uart4 */
		SUNXI_FUNCTION(0x7, "uart5"),         /* uart5 */
		SUNXI_FUNCTION(0x8, "spi1"),          /* spi1 */
		SUNXI_FUNCTION(0x9, "sdc2"),          /* sdc2 */
		SUNXI_FUNCTION(0xa, "pwm0_3"),        /* pwm0_3 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 4, 5),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(E, 6),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "ncsi0"),         /* ncsi0 */
		SUNXI_FUNCTION(0x3, "uart2"),         /* uart2 */
		SUNXI_FUNCTION(0x4, "lcd0"),          /* lcd0 */
		SUNXI_FUNCTION(0x5, "can"),           /* can */
		SUNXI_FUNCTION(0x6, "owa"),           /* owa */
		SUNXI_FUNCTION(0x7, "ir"),            /* ir */
		SUNXI_FUNCTION(0x8, "spi1"),          /* spi1 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 4, 6),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(E, 7),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "ncsi0"),         /* ncsi0 */
		SUNXI_FUNCTION(0x3, "uart2"),         /* uart2 */
		SUNXI_FUNCTION(0x4, "lcd0"),          /* lcd0 */
		SUNXI_FUNCTION(0x5, "can"),           /* can */
		SUNXI_FUNCTION(0x8, "spi1"),          /* spi1 */
		SUNXI_FUNCTION(0x9, "clk"),           /* clk */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 4, 7),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(E, 8),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "ncsi0"),         /* ncsi0 */
		SUNXI_FUNCTION(0x3, "uart2"),         /* uart2 */
		SUNXI_FUNCTION(0x4, "twi0"),          /* twi0 */
		SUNXI_FUNCTION(0x5, "can"),           /* can */
		SUNXI_FUNCTION(0x6, "uart3"),         /* uart3 */
		SUNXI_FUNCTION(0x7, "pwm0_0"),        /* pwm0_0 */
		SUNXI_FUNCTION(0x8, "spi1"),          /* spi1 */
		SUNXI_FUNCTION(0x9, "clk"),           /* clk */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 4, 8),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(E, 9),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "ncsi0"),         /* ncsi0 */
		SUNXI_FUNCTION(0x3, "uart2"),         /* uart2 */
		SUNXI_FUNCTION(0x4, "twi0"),          /* twi0 */
		SUNXI_FUNCTION(0x5, "can"),           /* can */
		SUNXI_FUNCTION(0x6, "uart3"),         /* uart3 */
		SUNXI_FUNCTION(0x7, "pwm0_1"),        /* pwm0_1 */
		SUNXI_FUNCTION(0x8, "spi1"),          /* spi1 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 4, 9),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(E, 10),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "ncsi0"),         /* ncsi0 */
		SUNXI_FUNCTION(0x3, "uart4"),         /* uart4 */
		SUNXI_FUNCTION(0x4, "twi1"),          /* twi1 */
		SUNXI_FUNCTION(0x5, "i2s0_mclk"),     /* i2s0_mclk */
		SUNXI_FUNCTION(0x6, "uart1"),         /* uart1 */
		SUNXI_FUNCTION(0x7, "pwm0_2"),        /* pwm0_2 */
		SUNXI_FUNCTION(0x8, "spi1"),          /* spi1 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 4, 10),  /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(E, 11),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "ncsi0"),         /* ncsi0 */
		SUNXI_FUNCTION(0x3, "uart4"),         /* uart4 */
		SUNXI_FUNCTION(0x4, "twi1"),          /* twi1 */
		SUNXI_FUNCTION(0x5, "i2s0_bclk"),     /* i2s0_bclk */
		SUNXI_FUNCTION(0x6, "uart1"),         /* uart1 */
		SUNXI_FUNCTION(0x7, "pwm0_3"),        /* pwm0_3 */
		SUNXI_FUNCTION(0x8, "spi1"),          /* spi1 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 4, 11),  /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(E, 12),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "ncsi0"),         /* ncsi0 */
		SUNXI_FUNCTION(0x3, "uart4"),         /* uart4 */
		SUNXI_FUNCTION(0x4, "uart5"),         /* uart5 */
		SUNXI_FUNCTION(0x5, "i2s0_lrck"),     /* i2s0_lrck */
		SUNXI_FUNCTION(0x6, "ir"),            /* ir */
		SUNXI_FUNCTION(0x7, "twi2"),          /* twi2 */
		SUNXI_FUNCTION(0x8, "spi1"),          /* spi1 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 4, 12),  /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(E, 13),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "pwm0_2"),        /* pwm0_2 */
		SUNXI_FUNCTION(0x3, "uart4"),         /* uart4 */
		SUNXI_FUNCTION(0x4, "uart5"),         /* uart5 */
		SUNXI_FUNCTION(0x5, "i2s0_din0"),     /* i2s0_din0 */
		SUNXI_FUNCTION(0x6, "i2s0_dout0"),    /* i2s0_dout0 */
		SUNXI_FUNCTION(0x7, "twi2"),          /* twi2 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 4, 13),  /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(E, 14),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "pwm0_3"),        /* pwm0_3 */
		SUNXI_FUNCTION(0x3, "ir"),            /* ir */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 4, 14),  /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	/* bank F */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(F, 0),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "sdc0"),          /* sdc0 */
		SUNXI_FUNCTION(0x3, "djtag"),         /* djtag */
		SUNXI_FUNCTION(0x4, "rjtag"),         /* rjtag */
		SUNXI_FUNCTION(0x5, "pwm0_0"),        /* pwm0_0 */
		SUNXI_FUNCTION(0x6, "uart1"),         /* uart1 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 5, 0),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(F, 1),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "sdc0"),          /* sdc0 */
		SUNXI_FUNCTION(0x3, "djtag"),         /* djtag */
		SUNXI_FUNCTION(0x4, "rjtag"),         /* rjtag */
		SUNXI_FUNCTION(0x5, "pwm0_1"),        /* pwm0_1 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 5, 1),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(F, 2),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "sdc0"),          /* sdc0 */
		SUNXI_FUNCTION(0x3, "uart0"),         /* uart0 */
		SUNXI_FUNCTION(0x4, "twi0"),          /* twi0 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 5, 2),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(F, 3),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "sdc0"),          /* sdc0 */
		SUNXI_FUNCTION(0x3, "djtag"),         /* djtag */
		SUNXI_FUNCTION(0x4, "rjtag"),         /* rjtag */
		SUNXI_FUNCTION(0x5, "twi1"),          /* twi1 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 5, 3),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(F, 4),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "sdc0"),          /* sdc0 */
		SUNXI_FUNCTION(0x3, "uart0"),         /* uart0 */
		SUNXI_FUNCTION(0x4, "twi0"),          /* twi0 */
		SUNXI_FUNCTION(0x6, "uart1"),         /* uart1 */
		SUNXI_FUNCTION(0x7, "twi2"),          /* twi2 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 5, 4),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(F, 5),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "sdc0"),          /* sdc0 */
		SUNXI_FUNCTION(0x3, "djtag"),         /* djtag */
		SUNXI_FUNCTION(0x4, "rjtag"),         /* rjtag */
		SUNXI_FUNCTION(0x5, "twi1"),          /* twi1 */
		SUNXI_FUNCTION(0x6, "uart1"),         /* uart1 */
		SUNXI_FUNCTION(0x7, "twi2"),          /* twi2 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 5, 5),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(F, 6),
		SUNXI_FUNCTION(0x0, "gpio_in"),       /* gpio_in */
		SUNXI_FUNCTION(0x1, "gpio_out"),      /* gpio_out */
		SUNXI_FUNCTION(0x2, "pwm0_3"),        /* pwm0_3 */
		SUNXI_FUNCTION(0x3, "ir"),            /* ir */
		SUNXI_FUNCTION(0x4, "spi0"),          /* spi0 */
		SUNXI_FUNCTION(0x5, "spif0"),         /* spif0 */
		SUNXI_FUNCTION(0x6, "uart1"),         /* uart1 */
		SUNXI_FUNCTION_IRQ_BANK(0xe, 5, 6),   /* eint */
		SUNXI_FUNCTION(0xf, "io_disabled")),  /* io_disabled */
};

static const unsigned int sun252i_f101_irq_bank_map[] = { 0, 1, 2, 3, 4, 5 };

static const struct sunxi_pinctrl_desc sun252i_f101_pinctrl_data = {
	.pins = sun252i_f101_pins,
	.npins = ARRAY_SIZE(sun252i_f101_pins),
	.irq_banks = ARRAY_SIZE(sun252i_f101_irq_bank_map),
	.irq_bank_map = sun252i_f101_irq_bank_map,
};

static int sun252i_f101_pinctrl_probe(struct platform_device *pdev)
{
	return sunxi_pinctrl_init_with_flags(pdev, &sun252i_f101_pinctrl_data,
					     SUNXI_PINCTRL_NEW_REG_LAYOUT);
}

static const struct of_device_id sun252i_f101_pinctrl_match[] = {
	{ .compatible = "allwinner,sun252i-f101-pinctrl" },
	{ }
};
MODULE_DEVICE_TABLE(of, sun252i_f101_pinctrl_match);

static struct platform_driver sun252i_f101_pinctrl_driver = {
	.probe = sun252i_f101_pinctrl_probe,
	.driver = {
		.name = "sun252i-f101-pinctrl",
		.of_match_table = sun252i_f101_pinctrl_match,
	},
};
builtin_platform_driver(sun252i_f101_pinctrl_driver);

MODULE_DESCRIPTION("Allwinner sun252i F101 pinctrl driver");
MODULE_LICENSE("GPL");
