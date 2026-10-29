// SPDX-License-Identifier: GPL-2.0

#include <linux/mtd/spi-nor.h>

#include "core.h"

/*
 * The SFDP of the P25Q128H is the first revision of JESD216: it tells that
 * DTR clocking is supported but not how the quad DTR read is made. That is
 * the 1-4-4 DTR read of the datasheet: 0xed, a mode byte and 7 wait states.
 */
static int puya_p25q128h_late_init(struct spi_nor *nor)
{
	struct spi_nor_flash_parameter *params = nor->params;

	if (!(params->hwcaps.mask & SNOR_HWCAPS_READ_1_4_4))
		return 0;

	params->hwcaps.mask |= SNOR_HWCAPS_READ_1_4_4_DTR;
	spi_nor_set_read_settings(&params->reads[SNOR_CMD_READ_1_4_4_DTR],
				  1, 7, SPINOR_OP_READ_1_4_4_DTR,
				  SNOR_PROTO_1_4_4_DTR);

	return 0;
}

static const struct spi_nor_fixups puya_p25q128h_fixups = {
	.late_init = puya_p25q128h_late_init,
};

static const struct flash_info puya_nor_parts[] = {
	{
		.id = SNOR_ID(0x85, 0x20, 0x18),
		.name = "p25q128h",
		.fixups = &puya_p25q128h_fixups,
	},
};

const struct spi_nor_manufacturer spi_nor_puya = {
	.name = "puya",
	.parts = puya_nor_parts,
	.nparts = ARRAY_SIZE(puya_nor_parts),
};
