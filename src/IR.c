#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#define IR_PIN 4

int main(void)
{
    const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    int val, last = -1;

    printk("\n=== IR Sensor Test ===\n");
    printk("OUT -> P0.04\n\n");

    gpio_pin_configure(gpio_dev, IR_PIN, GPIO_INPUT | GPIO_PULL_UP);

    while (1) {
        val = gpio_pin_get(gpio_dev, IR_PIN);
        if (val != last) {
            printk(val ? "Clear\n" : "DETECTED!\n");
            last = val;
        }
        k_msleep(50);
    }
    return 0;
}
