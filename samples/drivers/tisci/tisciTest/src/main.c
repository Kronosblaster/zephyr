#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/firmware/tisci/tisci.h>

void tisci_test(void)
{
    const struct device *dmsc = DEVICE_DT_GET(DT_NODELABEL(dmsc));
    if (!device_is_ready(dmsc)) {
        printk("DMSC device not ready!\n");
        return;
    }
    printk("TISCI test started!\n");

    // 1. Get revision
    struct tisci_version_info ver;
    int ret = tisci_cmd_get_revision(dmsc, &ver);
    if (ret == 0) {
        printk("TISCI revision: %u.%u, ABI: %u.%u, Description: %s\n",
               ver.abi_major, ver.abi_minor, ver.abi_major, ver.abi_minor, ver.firmware_description);
    } else {
        printk("Failed to get TISCI revision (err %d)\n", ret);
    }

    // 2. Get clock frequency
    uint64_t freq = 0;
    ret = tisci_cmd_clk_get_freq(dmsc, 152, 0, &freq);
    if (ret == 0) {
        printk("Clock 152 freq: %llu Hz\n", freq);
    } else {
        printk("Failed to get clock freq (err %d)\n", ret);
    }

    // 3. Set clock frequency
    uint64_t min_freq = 96000000, target_freq = 96000000, max_freq = 96000000;
    ret = tisci_cmd_clk_set_freq(dmsc, 152, 0, min_freq, target_freq, max_freq);
    if (ret == 0) {
        printk("Clock 152 freq set to %llu Hz\n", target_freq);
    } else {
        printk("Failed to set clock freq (err %d)\n", ret);
    }

    // 4. Get clock frequency again
    freq = 0;
    ret = tisci_cmd_clk_get_freq(dmsc, 152, 0, &freq);
    if (ret == 0) {
        printk("Clock 152 freq after set: %llu Hz\n", freq);
    } else {
        printk("Failed to get clock freq after set (err %d)\n", ret);
    }

    // 5. Get power domain device (turn on)
    ret = tisci_cmd_get_device(dmsc, 0);
    if (ret == 0) {
        printk("Power domain device 0 turned ON\n");
        // Check device state after turning ON using tisci_get_device_state
        uint32_t clcnt = 0, resets = 0;
        uint8_t p_state = 0, c_state = 0;
        int state_ret = tisci_get_device_state(dmsc, 0, &clcnt, &resets, &p_state, &c_state);
        if (state_ret == 0) {
            printk("Device 0 state after ON: clcnt=%u, resets=0x%x, p_state=0x%x, c_state=0x%x\n",
                   clcnt, resets, p_state, c_state);
        } else {
            printk("Failed to get device 0 state after ON (err %d)\n", state_ret);
        }
    } else {
        printk("Failed to turn ON power domain device 0 (err %d)\n", ret);
    }

    // 6. Put power domain device (turn off)
    ret = tisci_cmd_put_device(dmsc, 0);
    if (ret == 0) {
        printk("Power domain device 0 turned OFF\n");
        // Check device state after turning OFF using tisci_get_device_state
        uint32_t clcnt = 0, resets = 0;
        uint8_t p_state = 0, c_state = 0;
        int state_ret = tisci_get_device_state(dmsc, 0, &clcnt, &resets, &p_state, &c_state);
        if (state_ret == 0) {
            printk("Device 0 state after OFF: clcnt=%u, resets=0x%x, p_state=0x%x, c_state=0x%x\n",
                   clcnt, resets, p_state, c_state);
        } else {
            printk("Failed to get device 0 state after OFF (err %d)\n", state_ret);
        }
    } else {
        printk("Failed to turn OFF power domain device 0 (err %d)\n", ret);
    }
}

int main(void)
{
	tisci_test();
	return 0;
}
