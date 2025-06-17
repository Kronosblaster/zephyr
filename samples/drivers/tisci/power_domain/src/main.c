#include "zephyr/pm/device_runtime.h"
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <stdio.h>
#include <zephyr/pm/device.h>
#include <assert.h>
#include <zephyr/drivers/firmware/tisci/tisci.h>

#define POWER_DOMAIN_NODE DT_NODELABEL(adc0_pd)

static void print_tisci_state(const struct device *dmsc, uint32_t dev_id, const char *msg)
{
	uint32_t clcnt = 0, resets = 0;
	uint8_t p_state = 0, c_state = 0;
	int ret = tisci_get_device_state(dmsc, dev_id, &clcnt, &resets, &p_state, &c_state);
	assert(ret == 0);
	printf("[TISCI] %s: clcnt=%u, resets=0x%x, p_state=0x%x, c_state=0x%x\n", msg, clcnt,
	       resets, p_state, c_state);
}

int main(void)
{
	const struct device *pd_dev;
	int ret;

	pd_dev = DEVICE_DT_GET(POWER_DOMAIN_NODE);

	assert(pd_dev != NULL);

	if (!device_is_ready(pd_dev)) {
		printf("Power domain device not ready\n");
		assert(0 && "Device not ready");
		return -ENODEV;
	}
	const struct device *dmsc = DEVICE_DT_GET(DT_NODELABEL(dmsc));
	assert(dmsc != NULL && device_is_ready(dmsc));
	// Check initial state with TISCI before any power operation
	print_tisci_state(dmsc, 0, "Initial state");

	/* Test: Power on the device */
	ret = pm_device_runtime_get(pd_dev);
	assert(ret == 0);
	k_sleep(K_MSEC(100));
	printf("Power domain powered ON (test passed)\n");
	print_tisci_state(dmsc, 0, "After ON");

	/* Test: Power off the device */
	ret = pm_device_runtime_put(pd_dev);
	assert(ret == 0);
	k_sleep(K_MSEC(100));
	printf("Power domain powered OFF (test passed)\n");
	print_tisci_state(dmsc, 0, "After OFF");

	/* Test: Power on the device */
	ret = pm_device_runtime_get(pd_dev);
	assert(ret == 0);
	k_sleep(K_MSEC(100));
	printf("Power domain powered ON (test passed)\n");
	print_tisci_state(dmsc, 0, "After ON");

	/* Test: Power off again */
	ret = pm_device_runtime_put(pd_dev);
	assert(ret == 0);
	k_sleep(K_MSEC(100));
	printf("Power domain powered OFF again (test passed)\n");
	print_tisci_state(dmsc, 0, "After OFF again");

	printf("All power domain driver tests passed!\n");
	return 0;
}
