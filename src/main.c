// // Servo test code

// #include <zephyr/kernel.h>
// #include <zephyr/drivers/pwm.h>
// #include <zephyr/device.h>
// #include <zephyr/sys/printk.h>

// /* PWM controller */
// #define PWM_LED0 DT_ALIAS(pwm_led0)
// static const struct pwm_dt_spec pwm_led0 = PWM_DT_SPEC_GET(PWM_LED0);

// /* PWM settings */
// #define PWM_PERIOD          PWM_USEC(20000)  /* 20ms period = 50Hz */
// #define PWM_MIN_PULSE       PWM_USEC(900)    /* 0 degrees */
// #define PWM_CENTER_PULSE    PWM_USEC(1700)   /* 90 degrees */
// #define PWM_MAX_PULSE       PWM_USEC(2500)   /* 180 degrees */

// /* Set servo position */
// int servo_set_pulse(uint32_t pulse_ns)
// {
//     int err = pwm_set_dt(&pwm_led0, PWM_PERIOD, pulse_ns);
//     if (err) {
//         printk("PWM set failed: %d\n", err);
//     }
//     return err;
// }

// int main(void)
// {
//     printk("\n=== Servo Test - Automatic Cycling ===\n\n");
    
//     /* Check if PWM is ready */
//     if (!pwm_is_ready_dt(&pwm_led0)) {
//         printk("Error: PWM device not ready\n");
//         return 0;
//     }
    
//     printk("PWM ready! Starting servo test...\n\n");
    
//     /* Start at center */
//     printk("Moving to CENTER (90°)\n");
//     servo_set_pulse(PWM_CENTER_PULSE);
//     k_sleep(K_SECONDS(2));
    
//     /* Main test loop */
//     while (1) {
//         /* Move to 0 degrees */
//         printk("Moving to 0° (MIN)\n");
//         servo_set_pulse(PWM_MIN_PULSE);
//         k_sleep(K_SECONDS(3));
        
//         /* Move to 90 degrees */
//         printk("Moving to 90° (CENTER)\n");
//         servo_set_pulse(PWM_CENTER_PULSE);
//         k_sleep(K_SECONDS(3));
        
//         /* Move to 180 degrees */
//         printk("Moving to 180° (MAX)\n");
//         servo_set_pulse(PWM_MAX_PULSE);
//         k_sleep(K_SECONDS(3));
        
//         /* Back to center */
//         printk("Moving to 90° (CENTER)\n");
//         servo_set_pulse(PWM_CENTER_PULSE);
//         k_sleep(K_SECONDS(3));
        
//         printk("\n--- Cycle complete, repeating ---\n\n");
//     }
    
//     return 0;
// }
// ===================================================================
// Keypad tester
/*
 * Keypad driver for 4x4 matrix keypad
 * Pins remapped to avoid conflicts with servo (P0.07) and joystick (P0.05)
 */
/*
 * Keypad Test - 4 Buttons
 */

// #include <stdio.h>
// #include <zephyr/kernel.h>
// #include "keypad.h"

// int main(void)
// {
//     char key;

//     printf("\n");
//     printf("========================================\n");
//     printf("   4-Button Keypad Test\n");
//     printf("   nRF7002-DK\n");
//     printf("========================================\n\n");

//     if (keypad_init() < 0) {
//         printf("Keypad initialization failed!\n");
//         return 0;
//     }

//     printf("✓ Ready! Press buttons 1, 2, 3, or A\n\n");

//     while (1) {
//         key = keypad_scan();
        
//         if (key != 0) {
//             printf(">>> Button [%c] pressed! <<<\n", key);
//         }
        
//         k_msleep(50);
//     }

//     return 0;
// }


// ===================================================================
// Just the servo code


// #include <zephyr/kernel.h>
// #include <zephyr/drivers/pwm.h>
// #include <zephyr/device.h>
// #include <zephyr/logging/log.h>
// #include <dk_buttons_and_leds.h>

// LOG_MODULE_REGISTER(servo, LOG_LEVEL_INF);

// * PWM controller *
// #define PWM_LED0    DT_ALIAS(pwm_led0)
// static const struct pwm_dt_spec pwm_led0 = PWM_DT_SPEC_GET(PWM_LED0);

// /* PWM settings */
// #define PWM_PERIOD          PWM_USEC(40000)  /* 20ms period */
// #define PWM_MIN_PULSE_WIDTH PWM_USEC(900)   /* 1ms = 0 degrees */
// #define PWM_MAX_PULSE_WIDTH PWM_USEC(2500)   /* 2ms = 180 degrees */
// #define PWM_CHANNEL         0

// /* Function to set motor angle */
// int set_motor_angle(uint32_t pulse_width_ns)
// {
//     int err;
//     err = pwm_set_dt(&pwm_led0, PWM_PERIOD, pulse_width_ns);
//     if (err) {
//         LOG_ERR("pwm_set_dt_returned %d", err);
//     }
 
//     return err;
// }

// /* Button callback */
// static void button_handler(uint32_t button_state, uint32_t has_changed)
// {
//     int err;
    
//     switch (has_changed) {
//     case DK_BTN1_MSK:
//         if (button_state & DK_BTN1_MSK) {
//             LOG_INF("Button 1 pressed");
//             err = set_motor_angle(PWM_MIN_PULSE_WIDTH);
//         }
//         break;
//     case DK_BTN2_MSK:
//         if (button_state & DK_BTN2_MSK) {
//             LOG_INF("Button 2 pressed");
//             err = set_motor_angle(PWM_MAX_PULSE_WIDTH);
//         }
//         break;
//     }
// }

// int main(void)
// {
//     int err;
    
//     if (!pwm_is_ready_dt(&pwm_led0)) {
//     LOG_ERR("Error: PWM device %s is not ready", pwm_led0.dev->name);
//     return 0;
//     }
//     err = dk_buttons_init(button_handler);

//     err = pwm_set_dt(&pwm_led0, PWM_PERIOD, PWM_MIN_PULSE_WIDTH);
//     if (err) {
//     LOG_ERR("pwm_set_dt returned %d", err);
//     return 0;
//     }
// }


// ==================================================

// Joystick + servo code

// #include <zephyr/kernel.h>
// #include <zephyr/drivers/pwm.h>
// #include <zephyr/drivers/adc.h>
// #include <zephyr/device.h>
// #include <zephyr/sys/printk.h>

// /* PWM controller for Servo */
// #define PWM_LED0 DT_ALIAS(pwm_led0)
// static const struct pwm_dt_spec pwm_led0 = PWM_DT_SPEC_GET(PWM_LED0);

// /* PWM settings - Extended range for more travel */
// #define PWM_PERIOD       PWM_USEC(20000)  /* 20ms = 50Hz */
// #define PWM_MIN_PULSE    PWM_USEC(500)    /* 0 degrees */
// #define PWM_MAX_PULSE    PWM_USEC(2800)   /* 180 degrees */

// /* ADC for Joystick */
// #define ADC_NODE DT_NODELABEL(adc)
// #define VRX_CHANNEL 1  /* P0.05 = AIN1 */

// static const struct device *adc_dev;

// /* Joystick Calibration - Adjust these to match YOUR joystick's actual range! */
// #define JOYSTICK_MIN     300   /* Joystick pushed fully LEFT */
// #define JOYSTICK_CENTER  2048  /* Joystick at rest (center) */
// #define JOYSTICK_MAX     3800  /* Joystick pushed fully RIGHT */

// /* ADC configuration */
// static const struct adc_channel_cfg vrx_channel_cfg = {
//     .gain = ADC_GAIN_1_6,
//     .reference = ADC_REF_INTERNAL,
//     .acquisition_time = ADC_ACQ_TIME_DEFAULT,
//     .channel_id = VRX_CHANNEL,
//     .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput1,
// };

// static int16_t sample_buffer;

// static const struct adc_sequence sequence = {
//     .channels = BIT(VRX_CHANNEL),
//     .buffer = &sample_buffer,
//     .buffer_size = sizeof(sample_buffer),
//     .resolution = 12,
// };

// /* Set servo pulse width */
// int servo_set_pulse(uint32_t pulse_ns)
// {
//     int err = pwm_set_dt(&pwm_led0, PWM_PERIOD, pulse_ns);
//     if (err) {
//         printk("PWM set failed: %d\n", err);
//     }
//     return err;
// }

// /* Read joystick X-axis */
// int read_joystick_x(void)
// {
//     int ret = adc_read(adc_dev, &sequence);
//     if (ret) {
//         printk("ADC read failed\n");
//         return JOYSTICK_CENTER; /* Center on error */
//     }
//     return sample_buffer;
// }

// /* Constrain value between min and max */
// int constrain(int value, int min_val, int max_val)
// {
//     if (value < min_val) return min_val;
//     if (value > max_val) return max_val;
//     return value;
// }

// /* Map ADC value to servo pulse - using calibrated joystick range */
// uint32_t map_to_servo_pulse(int adc_value)
// {
//     /* Constrain ADC to joystick's actual range */
//     adc_value = constrain(adc_value, JOYSTICK_MIN, JOYSTICK_MAX);
    
//     /* Map joystick range to full servo range */
//     uint32_t pulse = PWM_MIN_PULSE + 
//                      ((adc_value - JOYSTICK_MIN) * (PWM_MAX_PULSE - PWM_MIN_PULSE)) / 
//                      (JOYSTICK_MAX - JOYSTICK_MIN);
    
//     return pulse;
// }

// /* Map ADC to angle for display */
// int map_to_angle(int adc_value)
// {
//     adc_value = constrain(adc_value, JOYSTICK_MIN, JOYSTICK_MAX);
//     return ((adc_value - JOYSTICK_MIN) * 180) / (JOYSTICK_MAX - JOYSTICK_MIN);
// }

// int main(void)
// {
//     int ret;
    
//     printk("\n=== Joystick Real-Time Servo Control ===\n");
//     printk("Calibrated for FULL servo range!\n");
//     printk("Joystick range: %d to %d\n\n", JOYSTICK_MIN, JOYSTICK_MAX);
    
//     /* Initialize PWM */
//     if (!pwm_is_ready_dt(&pwm_led0)) {
//         printk("PWM device not ready\n");
//         return 0;
//     }
//     printk("PWM ready\n");
    
//     /* Initialize ADC */
//     adc_dev = DEVICE_DT_GET(ADC_NODE);
//     if (!device_is_ready(adc_dev)) {
//         printk("ADC device not ready\n");
//         return 0;
//     }
//     printk("ADC ready\n");
    
//     /* Configure ADC channel */
//     ret = adc_channel_setup(adc_dev, &vrx_channel_cfg);
//     if (ret) {
//         printk("ADC setup failed: %d\n", ret);
//         return 0;
//     }
//     printk("ADC channel configured\n\n");
    
//     /* Start at center */
//     servo_set_pulse(PWM_USEC(1700));
//     k_sleep(K_MSEC(500));
    
//     printk("Ready! Move the joystick to EXTREMES...\n");
//     printk("Watch the ADC values to calibrate if needed\n\n");
    
//     /* Main control loop - fast response! */
//     while (1) {
//         /* Read joystick position */
//         int vrx = read_joystick_x();
        
//         /* Map to servo pulse width */
//         uint32_t servo_pulse = map_to_servo_pulse(vrx);
        
//         /* Set servo immediately */
//         servo_set_pulse(servo_pulse);
        
//         /* Calculate angle */
//         int angle = map_to_angle(vrx);
        
//         /* Print status */
//         printk("ADC: %4d | Angle: %3d° | Pulse: %4d us | ", 
//                vrx, angle, servo_pulse / 1000);
        
//         /* Direction indicator */
//         if (vrx < (JOYSTICK_MIN + 200)) {
//             printk("<<< FULL LEFT\n");
//         } else if (vrx > (JOYSTICK_MAX - 200)) {
//             printk("FULL RIGHT >>>\n");
//         } else if (vrx < (JOYSTICK_CENTER - 200)) {
//             printk("<< Left\n");
//         } else if (vrx > (JOYSTICK_CENTER + 200)) {
//             printk("Right >>\n");
//         } else {
//             printk("CENTER\n");
//         }
        
//         /* Fast update - 20ms = 50Hz for smooth tracking */
//         k_msleep(20);
//     }
    
//     return 0;
// }


// =================================================================================
// // Servo + Joystick + Keypad
#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include "keypad.h"

/* PWM controller for Servo */
#define PWM_LED0 DT_ALIAS(pwm_led0)
static const struct pwm_dt_spec pwm_led0 = PWM_DT_SPEC_GET(PWM_LED0);

#define PWM_PERIOD       PWM_USEC(40000)  /* 20ms = 50Hz ✓ */
#define PWM_MIN_PULSE    PWM_USEC(500)    /* 0° (extended range) ✓ */
#define PWM_MAX_PULSE    PWM_USEC(2500)   /* 180° (extended range) ✓ */

/* OR for standard range: */
// #define PWM_MIN_PULSE    PWM_USEC(1000)   /* 0° (standard) ✓ */
// #define PWM_MAX_PULSE    PWM_USEC(2000) 

/* ADC for Joystick */
#define ADC_NODE DT_NODELABEL(adc)
#define VRX_CHANNEL 1  /* P0.05 = AIN1 */

static const struct device *adc_dev;

/* Joystick Calibration - Adjust these to match YOUR joystick's actual range! */
#define JOYSTICK_MIN     300   /* Joystick pushed fully LEFT */
#define JOYSTICK_CENTER  2048  /* Joystick at rest (center) */
#define JOYSTICK_MAX     3800  /* Joystick pushed fully RIGHT */

/* ADC configuration */
static const struct adc_channel_cfg vrx_channel_cfg = {
    .gain = ADC_GAIN_1_6,
    .reference = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id = VRX_CHANNEL,
    .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput1,
};

static int16_t sample_buffer;

static const struct adc_sequence sequence = {
    .channels = BIT(VRX_CHANNEL),
    .buffer = &sample_buffer,
    .buffer_size = sizeof(sample_buffer),
    .resolution = 12,
};

/* Servo lock state */
static bool servo_locked = false;
static uint32_t locked_pulse = PWM_USEC(2500);  /* Locked position */

/* Set servo pulse width */
int servo_set_pulse(uint32_t pulse_ns)
{
    int err = pwm_set_dt(&pwm_led0, PWM_PERIOD, pulse_ns);
    if (err) {
        printk("PWM set failed: %d\n", err);
    }
    return err;
}

/* Read joystick X-axis */
int read_joystick_x(void)
{
    int ret = adc_read(adc_dev, &sequence);
    if (ret) {
        printk("ADC read failed\n");
        return JOYSTICK_CENTER; /* Center on error */
    }
    return sample_buffer;
}

/* Constrain value between min and max */
int constrain(int value, int min_val, int max_val)
{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

/* Map ADC value to servo pulse - using calibrated joystick range */
uint32_t map_to_servo_pulse(int adc_value)
{
    /* Constrain ADC to joystick's actual range */
    adc_value = constrain(adc_value, JOYSTICK_MIN, JOYSTICK_MAX);
    
    /* Map joystick range to full servo range */
    uint32_t pulse = PWM_MIN_PULSE + 
                     ((adc_value - JOYSTICK_MIN) * (PWM_MAX_PULSE - PWM_MIN_PULSE)) / 
                     (JOYSTICK_MAX - JOYSTICK_MIN);
    
    return pulse;
}

/* Map ADC to angle for display */
int map_to_angle(int adc_value)
{
    adc_value = constrain(adc_value, JOYSTICK_MIN, JOYSTICK_MAX);
    return ((adc_value - JOYSTICK_MIN) * 180) / (JOYSTICK_MAX - JOYSTICK_MIN);
}

int main(void)
{
    int ret;
    
    printk("\n╔════════════════════════════════════════╗\n");
    printk("║ Joystick + Keypad Servo Control      ║\n");
    printk("╚════════════════════════════════════════╝\n\n");
    printk("Calibrated for FULL servo range!\n");
    printk("Joystick range: %d to %d\n", JOYSTICK_MIN, JOYSTICK_MAX);
    printk("Button '1': Lock servo\n");
    printk("Button 'A': Unlock servo\n\n");
    
    /* Initialize PWM */
    if (!pwm_is_ready_dt(&pwm_led0)) {
        printk("PWM device not ready\n");
        return 0;
    }
    printk("✓ PWM ready\n");
    
    /* Initialize ADC */
    adc_dev = DEVICE_DT_GET(ADC_NODE);
    if (!device_is_ready(adc_dev)) {
        printk("ADC device not ready\n");
        return 0;
    }
    printk("✓ ADC ready\n");
    
    /* Configure ADC channel */
    ret = adc_channel_setup(adc_dev, &vrx_channel_cfg);
    if (ret) {
        printk("ADC setup failed: %d\n", ret);
        return 0;
    }
    printk("✓ ADC configured\n");
    
    /* Initialize keypad */
    ret = keypad_init();
    if (ret) {
        printk("Keypad init failed: %d\n", ret);
        return 0;
    }
    
    /* Start at center */
    servo_set_pulse(PWM_USEC(1700));
    k_sleep(K_MSEC(500));
    
    printk("\nReady! Status: UNLOCKED\n");
    printk("═══════════════════════════════════════════\n\n");
    
    /* Main control loop - fast response! */
    while (1) {
        /* Scan keypad */
        char key = keypad_scan();
        
        /* Process keypad buttons */
        if (key == '1' && !servo_locked) {
            /* Lock servo at current position */
            int vrx = read_joystick_x();
            locked_pulse = map_to_servo_pulse(vrx);
            servo_locked = true;
            
            int angle = map_to_angle(vrx);
            printk("\n🔒 SERVO LOCKED at %d us (%d°)\n", locked_pulse / 1000, angle);
            printk("Press 'A' to unlock\n\n");
        }
        else if (key == 'A' && servo_locked) {
            /* Unlock servo */
            servo_locked = false;
            printk("\n🔓 SERVO UNLOCKED - Joystick active\n\n");
        }
        
        if (!servo_locked) {
            /* UNLOCKED MODE - Joystick controls servo */
            int vrx = read_joystick_x();
            
            /* Map to servo pulse width */
            uint32_t servo_pulse = map_to_servo_pulse(vrx);
            
            /* Set servo immediately */
            servo_set_pulse(servo_pulse);
            
            /* Calculate angle */
            int angle = map_to_angle(vrx);
            
            /* Print status */
            printk("ADC: %4d | Angle: %3d° | Pulse: %4d us | ", 
                   vrx, angle, servo_pulse / 1000);
            
            /* Direction indicator */
            if (vrx < (JOYSTICK_MIN + 200)) {
                printk("<<< FULL LEFT\n");
            } else if (vrx > (JOYSTICK_MAX - 200)) {
                printk("FULL RIGHT >>>\n");
            } else if (vrx < (JOYSTICK_CENTER - 200)) {
                printk("<< Left\n");
            } else if (vrx > (JOYSTICK_CENTER + 200)) {
                printk("Right >>\n");
            } else {
                printk("CENTER\n");
            }
        }
        else {
            /* LOCKED MODE - Servo stays at locked position */
            servo_set_pulse(locked_pulse);
            
            /* Still read joystick but don't use it */
            read_joystick_x();
        }
        
        /* Fast update - 20ms = 50Hz for smooth tracking */
        k_msleep(20);
    }
    
    return 0;
}