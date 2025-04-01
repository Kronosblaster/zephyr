/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <stdio.h>
#include <zephyr/drivers/firmware/ti_sci/ti_sci.h>

int tisci_init(const struct device *dev);
int main(void)
{
	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);
	const struct device* dmsc = DEVICE_DT_GET(DT_NODELABEL(dmsc));
	ti_sci_cmd_get_revision(dmsc);
	const struct tisci_data *data = (const struct tisci_data *)(dmsc->data);
	struct ti_sci_version_info *ver;
	ver = &data->version;
	printf("SYSFW ABI: %d.%d (firmware rev 0x%04x '%s')\n", ver->abi_major,
	       ver->abi_minor, ver->firmware_revision, ver->firmware_description);
	return 0;
}
