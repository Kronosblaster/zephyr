#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <stdio.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/tisci_clock_control.h>
#include <assert.h>

int main(void)
{
    const struct device *clock_dev;
    struct tisci_clock_config req = TISCI_GET_CLOCK_DETAILS(uart1);
    uint64_t rate = 0;
    uint32_t rate32 = 0;
    int ret;

    clock_dev = TISCI_GET_CLOCK(uart1);
    assert(clock_dev != NULL);

    /* Test: Get current clock rate */
    ret = clock_control_get_rate(clock_dev, &req, &rate32);
    assert(ret == 0);
    printf("Current clock rate: %u\n", rate32);

    /* Test: Set clock rate to 96 MHz */
    rate = 96000000;
    ret = clock_control_set_rate(clock_dev, &req, &rate);
    assert(ret == 0);
    printf("Clock rate set to 96000000\n");

    /* Test: Set clock rate to 48 MHz */
    rate = 48000000;
    ret = clock_control_set_rate(clock_dev, &req, &rate);
    assert(ret == 0);
    printf("Clock rate set to 48000000\n");

    /* Test: Get clock rate after setting */
    ret = clock_control_get_rate(clock_dev, &req, &rate32);
    assert(ret == 0);
    printf("Current clock rate after set: %u\n", rate32);

    printf("All clock controller tests passed!\n");
    return 0;
}