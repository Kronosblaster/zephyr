#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <stdio.h>
#include <zephyr/pm/device.h>
#include <assert.h>

#define POWER_DOMAIN_NODE DT_NODELABEL(adc0_pd)

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

    /* Test: Power off the device */
    ret = pm_device_action_run(pd_dev, PM_DEVICE_ACTION_SUSPEND);
    assert(ret == 0);
    printf("Power domain powered OFF (test passed)\n");

    /* Test: Power on the device */
    ret = pm_device_action_run(pd_dev, PM_DEVICE_ACTION_RESUME);
    assert(ret == 0);
    printf("Power domain powered ON (test passed)\n");

    /* Test: Power off again */
    ret = pm_device_action_run(pd_dev, PM_DEVICE_ACTION_SUSPEND);
    assert(ret == 0);
    printf("Power domain powered OFF again (test passed)\n");

    printf("All power domain driver tests passed!\n");
    return 0;
}