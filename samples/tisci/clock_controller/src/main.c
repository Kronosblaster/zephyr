#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <stdio.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/tisci_clock_control.h>

int main(void)
{
	const struct device *clock_dev;
	uint64_t rate = 0;

	clock_dev = DEVICE_DT_GET(DT_PHANDLE(DT_NODELABEL(uart0), clocks));
	struct clock_config req = {.dev_id = DT_CLOCKS_CELL(DT_NODELABEL(uart0), devid),
				   .clk_id = DT_CLOCKS_CELL(DT_NODELABEL(uart0), clkid)};
	if (!clock_control_get_rate(clock_dev, &req, &rate)) {
		printf("\nCurrent clock rate is:%llu\n", rate);
	}
	rate = 96000000;
	if (!clock_control_set_rate(clock_dev, &req, &rate)) {
		printf("Clock rate 96000000 makes this unreadable");
	}
	rate = 48000000;
	if (!clock_control_set_rate(clock_dev, &req, &rate)) {
		printf("\nClock rate 48000000 makes this readable");
	}
	if (!clock_control_get_rate(clock_dev, &req, &rate)) {
		printf("\nCurrent clock rate is:%llu\n", rate);
	}
	if (!clock_control_off(clock_dev, &req)) {
		printf("\nClock is off\n");
	} else {
		printf("\nClock is on\n");
	}
	if (!clock_control_on(clock_dev, &req)) {
		printf("\nClock is on\n");
	} else {
		printf("\nClock is off\n");
	}
	return 0;
}