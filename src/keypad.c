#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
#include "keypad.h"

/* GPIO devices */
static const struct device *gpio0_dev;
static const struct device *gpio1_dev;

/* Pin definitions - matching overlay documentation */
#define ROW0_PIN 4     // P0.04 - Brown wire
#define COL0_PIN 25    // P0.25 - Yellow wire
#define COL1_PIN 26    // P0.26 - Blue wire
#define COL2_PIN 8     // P1.08 - Green wire
#define COL3_PIN 9     // P1.09 - Purple wire

/* Only Row 0 buttons work */
static const char keypad_buttons[4] = {'1', '2', '3', 'A'};

int keypad_init(void)
{
    int ret;
    
    printf("Initializing keypad...\n");
    printf("Wire colors (left to right): Brown, Black, White, Gray, Yellow, Blue, Green, Purple\n");
    printf("Only Row 0 working: buttons 1, 2, 3, A\n\n");
    
    gpio0_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(gpio0_dev)) {
        printf("ERROR: GPIO0 not ready\n");
        return -1;
    }

    gpio1_dev = DEVICE_DT_GET(DT_NODELABEL(gpio1));
    if (!device_is_ready(gpio1_dev)) {
        printf("ERROR: GPIO1 not ready\n");
        return -1;
    }

    /* Configure Row 0 as output */
    ret = gpio_pin_configure(gpio0_dev, ROW0_PIN, GPIO_OUTPUT_LOW);
    if (ret < 0) {
        printf("ERROR: Failed to configure row 0\n");
        return ret;
    }

    /* Configure all 4 column pins as inputs with pull-up */
    ret = gpio_pin_configure(gpio0_dev, COL0_PIN, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) return ret;
    
    ret = gpio_pin_configure(gpio0_dev, COL1_PIN, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) return ret;
    
    ret = gpio_pin_configure(gpio1_dev, COL2_PIN, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) return ret;
    
    ret = gpio_pin_configure(gpio1_dev, COL3_PIN, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) return ret;
    
    printf("✓ Keypad initialized successfully\n");
    printf("  Brown wire (Row 0) → P0.04\n");
    printf("  Yellow wire (Col 0) → P0.25\n");
    printf("  Blue wire (Col 1) → P0.26\n");
    printf("  Green wire (Col 2) → P1.08\n");
    printf("  Purple wire (Col 3) → P1.09\n\n");
    
    return 0;
}

char keypad_scan(void)
{
    static char last_key = 0;
    static bool last_pressed = false;
    char current_key = 0;
    bool key_pressed = false;

    /* Set Row 0 LOW to scan */
    gpio_pin_set(gpio0_dev, ROW0_PIN, 0);
    k_busy_wait(100);

    /* Check each column */
    if (gpio_pin_get(gpio0_dev, COL0_PIN) == 0) {
        current_key = '1';
        key_pressed = true;
    }
    else if (gpio_pin_get(gpio0_dev, COL1_PIN) == 0) {
        current_key = '2';
        key_pressed = true;
    }
    else if (gpio_pin_get(gpio1_dev, COL2_PIN) == 0) {
        current_key = '3';
        key_pressed = true;
    }
    else if (gpio_pin_get(gpio1_dev, COL3_PIN) == 0) {
        current_key = 'A';
        key_pressed = true;
    }

    /* Set Row 0 HIGH */
    gpio_pin_set(gpio0_dev, ROW0_PIN, 1);

    /* Debouncing */
    if (key_pressed && !last_pressed) {
        last_pressed = true;
        last_key = current_key;
        return current_key;
    } else if (!key_pressed) {
        last_pressed = false;
        last_key = 0;
    }

    return 0;
}