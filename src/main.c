/*
 * NPM1300 Simple Test Program - Quick Verification
 * Updated for nrf54l15 with proper GPIO control
 * 
 * This simplified version allows manual testing of each feature
 * without button controls. Just uncomment the test you want to run.
 */

#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/mfd/npm1300.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>

#define TEST_DELAY_MS 2000

/* Device handles */
static const struct device *pmic = DEVICE_DT_GET(DT_NODELABEL(npm1300_ek_pmic));
static const struct device *leds = DEVICE_DT_GET(DT_NODELABEL(npm1300_ek_leds));
static const struct device *regulators = DEVICE_DT_GET(DT_NODELABEL(npm1300_ek_regulators));

/* GPIO controller - this is the npm1300 GPIO controller */
static const struct device *npm_gpio = DEVICE_DT_GET(DT_NODELABEL(npm1300_ek_gpio));

/* For load switches */
static const struct device *ldsw1 = DEVICE_DT_GET(DT_NODELABEL(npm1300_ek_ldo1));
static const struct device *ldsw2 = DEVICE_DT_GET(DT_NODELABEL(npm1300_ek_ldo2));

/* For VOUT1 */
static const struct device *vout1 = DEVICE_DT_GET(DT_NODELABEL(npm1300_ek_buck1));

/* Test selection - uncomment ONE test at a time */
// #define TEST_RGB_LEDS_ONLY    // Only RGB LEDs blink
#define TEST_ALL_TOGETHER     // LEDs + GPIO1,2,3 + LS1,2 + VOUT1 all toggle together
// #define TEST_VOUT1            // VOUT1 enable test (stays on)
//#define TEST_ALL_SEQUENTIAL   // Run all tests one after another

void test_rgb_leds(void)
{
    printk("\n=== RGB LED Test ===\n");
    
    if (!device_is_ready(leds)) {
        printk("ERROR: LED device not ready\n");
        return;
    }
    
    for (int i = 0; i < 5; i++) {
        printk("Cycle %d: LEDs ON\n", i + 1);
        led_on(leds, 0U);  // Red
        led_on(leds, 1U);  // Green
        led_on(leds, 2U);  // Blue
        k_msleep(TEST_DELAY_MS);
        
        printk("Cycle %d: LEDs OFF\n", i + 1);
        led_off(leds, 0U);
        led_off(leds, 1U);
        led_off(leds, 2U);
        k_msleep(TEST_DELAY_MS);
    }
    
    printk("✓ RGB LED test complete\n");
}

void test_all_together(void)
{
    printk("\n=== ALL TOGETHER Test ===\n");
    printk("RGB LEDs + GPIO1,2,3 + LS1,2 + VOUT1 toggling synchronously\n");
    printk("Measure: GPIO1, GPIO2, GPIO3 pins, PVDD1, PVDD2, and C15 (VOUT1) voltages\n\n");
    
    /* Check all devices */
    if (!device_is_ready(leds)) {
        printk("ERROR: LED device not ready\n");
        return;
    }
    
    if (!device_is_ready(npm_gpio)) {
        printk("ERROR: npm1300 GPIO controller not ready\n");
        return;
    }
    
    if (!device_is_ready(ldsw1)) {
        printk("ERROR: LS1 device not ready\n");
        return;
    }
    
    if (!device_is_ready(ldsw2)) {
        printk("ERROR: LS2 device not ready\n");
        return;
    }
    
    if (!device_is_ready(vout1)) {
        printk("ERROR: VOUT1 device not ready\n");
        return;
    }
    
    /* Configure GPIOs as outputs */
    int ret;
    ret = gpio_pin_configure(npm_gpio, 1, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        printk("ERROR: Failed to configure GPIO1 (error: %d)\n", ret);
        return;
    }
    
    ret = gpio_pin_configure(npm_gpio, 2, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        printk("ERROR: Failed to configure GPIO2 (error: %d)\n", ret);
        return;
    }
    
    ret = gpio_pin_configure(npm_gpio, 3, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        printk("ERROR: Failed to configure GPIO3 (error: %d)\n", ret);
        return;
    }
    
    /* Set VOUT1 voltage to 3V */
    ret = regulator_set_voltage(vout1, 3000000, 3000000);
    if (ret < 0) {
        printk("ERROR: Failed to set VOUT1 voltage (error: %d)\n", ret);
        return;
    }
    
    printk("✓ All devices configured successfully\n\n");
    
    /* Toggle all together in sync */
    for (int i = 0; i < 5; i++) {
        printk("Cycle %d: ALL ON (LEDs + GPIO1,2,3 + LS1,2 + VOUT1)\n", i + 1);
        
        /* Turn on LEDs */
        led_on(leds, 0U);  // Red
        led_on(leds, 1U);  // Green
        led_on(leds, 2U);  // Blue
        
        /* Set GPIOs HIGH */
        gpio_pin_set(npm_gpio, 1, 1);
        gpio_pin_set(npm_gpio, 2, 1);
        gpio_pin_set(npm_gpio, 3, 1);
        
        /* Enable Load Switches */
        regulator_enable(ldsw1);
        regulator_enable(ldsw2);
        
        /* Enable VOUT1 */
        regulator_enable(vout1);
        
        k_msleep(TEST_DELAY_MS);
        
        printk("Cycle %d: ALL OFF\n", i + 1);
        
        /* Turn off LEDs */
        led_off(leds, 0U);
        led_off(leds, 1U);
        led_off(leds, 2U);
        
        /* Set GPIOs LOW */
        gpio_pin_set(npm_gpio, 1, 0);
        gpio_pin_set(npm_gpio, 2, 0);
        gpio_pin_set(npm_gpio, 3, 0);
        
        /* Disable Load Switches */
        regulator_disable(ldsw1);
        regulator_disable(ldsw2);
        
        /* Disable VOUT1 */
        regulator_disable(vout1);
        
        k_msleep(TEST_DELAY_MS);
    }
    
    printk("✓ ALL TOGETHER test complete\n");
}

void test_gpio1_gpio2(void)
{
    printk("\n=== GPIO1 & GPIO2 Test ===\n");
    printk("Measure voltage at GPIO1 and GPIO2 pins\n");
    
    if (!device_is_ready(npm_gpio)) {
        printk("ERROR: npm1300 GPIO controller not ready\n");
        return;
    }
    
    /* Configure GPIO1 (pin 1) and GPIO2 (pin 2) as outputs */
    int ret;
    ret = gpio_pin_configure(npm_gpio, 1, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        printk("ERROR: Failed to configure GPIO1 (error: %d)\n", ret);
        return;
    }
    
    ret = gpio_pin_configure(npm_gpio, 2, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        printk("ERROR: Failed to configure GPIO2 (error: %d)\n", ret);
        return;
    }
    
    for (int i = 0; i < 5; i++) {
        printk("Cycle %d: GPIO1 & GPIO2 HIGH\n", i + 1);
        gpio_pin_set(npm_gpio, 1, 1);
        gpio_pin_set(npm_gpio, 2, 1);
        k_msleep(TEST_DELAY_MS);
        
        printk("Cycle %d: GPIO1 & GPIO2 LOW\n", i + 1);
        gpio_pin_set(npm_gpio, 1, 0);
        gpio_pin_set(npm_gpio, 2, 0);
        k_msleep(TEST_DELAY_MS);
    }
    
    printk("✓ GPIO1/GPIO2 test complete\n");
}

void test_gpio3(void)
{
    printk("\n=== GPIO3 Test ===\n");
    printk("Measure: EN_BIO test point & V_BIO at C19\n");
    
    if (!device_is_ready(npm_gpio)) {
        printk("ERROR: npm1300 GPIO controller not ready\n");
        return;
    }
    
    /* Configure GPIO3 (pin 3) as output */
    int ret = gpio_pin_configure(npm_gpio, 3, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        printk("ERROR: Failed to configure GPIO3 (error: %d)\n", ret);
        return;
    }
    
    for (int i = 0; i < 5; i++) {
        printk("Cycle %d: GPIO3 HIGH\n", i + 1);
        gpio_pin_set(npm_gpio, 3, 1);
        k_msleep(TEST_DELAY_MS);
        
        printk("Cycle %d: GPIO3 LOW\n", i + 1);
        gpio_pin_set(npm_gpio, 3, 0);
        k_msleep(TEST_DELAY_MS);
    }
    
    printk("✓ GPIO3 test complete\n");
}

void test_vout1_enable(void)
{
    printk("\n=== VOUT1 Enable Test ===\n");
    printk("Measure: 3V at C15\n");
    
    if (!device_is_ready(vout1)) {
        printk("ERROR: VOUT1 device not ready\n");
        return;
    }
    
    int ret = regulator_set_voltage(vout1, 3000000, 3000000);
    if (ret < 0) {
        printk("ERROR: Failed to set voltage (error: %d)\n", ret);
        return;
    }
    
    ret = regulator_enable(vout1);
    if (ret < 0) {
        printk("ERROR: Failed to enable VOUT1 (error: %d)\n", ret);
        return;
    }
    
    printk("✓ VOUT1 enabled at 3V - measure at C15\n");
    printk("Holding for 5 seconds for measurement...\n");
    k_msleep(5000);
    printk("✓ VOUT1 test complete (still enabled)\n");
}

void test_ls1(void)
{
    printk("\n=== Load Switch 1 Test ===\n");
    printk("Measure: Voltage at PVDD1\n");
    
    if (!device_is_ready(ldsw1)) {
        printk("ERROR: LS1 device not ready\n");
        return;
    }
    
    for (int i = 0; i < 5; i++) {
        printk("LS1 ENABLE\n");
        regulator_enable(ldsw1);
        k_msleep(TEST_DELAY_MS);
        
        printk("LS1 DISABLE\n");
        regulator_disable(ldsw1);
        k_msleep(TEST_DELAY_MS);
    }
    
    printk("✓ LS1 test complete\n");
}

void test_ls2(void)
{
    printk("\n=== Load Switch 2 Test ===\n");
    printk("Measure: Voltage at PVDD2\n");
    
    if (!device_is_ready(ldsw2)) {
        printk("ERROR: LS2 device not ready\n");
        return;
    }
    
    for (int i = 0; i < 5; i++) {
        printk("LS2 ENABLE\n");
        regulator_enable(ldsw2);
        k_msleep(TEST_DELAY_MS);
        
        printk("LS2 DISABLE\n");
        regulator_disable(ldsw2);
        k_msleep(TEST_DELAY_MS);
    }
    
    printk("✓ LS2 test complete\n");
}

int main(void)
{
    printk("\n========================================\n");
    printk("  NPM1300 Simple Test Program\n");
    printk("  Version: 1.3 (Combined Tests)\n");
    printk("========================================\n\n");
    
    /* Test 1: I2C Communication */
    printk("TEST 1: I2C Communication\n");
    if (device_is_ready(pmic)) {
        printk("✓ PMIC device ready - I2C OK\n");
    } else {
        printk("✗ PMIC device not ready - I2C FAILED\n");
        return 0;
    }
    
    /* Check GPIO controller */
    if (device_is_ready(npm_gpio)) {
        printk("✓ npm1300 GPIO controller ready\n");
    } else {
        printk("⚠ npm1300 GPIO controller not ready (GPIO tests will fail)\n");
    }
    
    k_msleep(1000);
    
    /* Run selected test(s) */
#ifdef TEST_RGB_LEDS_ONLY
    printk("\n=== Running RGB LED Test ONLY (Continuously) ===\n");
    printk("Press reset to stop\n\n");
    while (1) {
        test_rgb_leds();
        k_msleep(1000);
    }
#endif

#ifdef TEST_ALL_TOGETHER
    printk("\n=== Running ALL TOGETHER Test (Continuously) ===\n");
    printk("LEDs + GPIO1,2,3 + LS1,2 + VOUT1 toggling synchronously\n");
    printk("Press reset to stop\n\n");
    while (1) {
        test_all_together();
        k_msleep(1000);
    }
#endif

#ifdef TEST_VOUT1
    printk("\n=== Running VOUT1 Test Once ===\n");
    printk("VOUT1 will remain enabled\n\n");
    test_vout1_enable();
    printk("\n=== VOUT1 Test Complete - Staying ON ===\n");
    printk("Measure voltage at C15. Press reset to exit.\n");
    /* Stay alive to keep VOUT1 enabled */
    while (1) {
        k_msleep(10000);
    }
#endif

#ifdef TEST_ALL_SEQUENTIAL
    printk("\n=== Running All Tests Sequentially ===\n");
    k_msleep(1000);
    
    test_rgb_leds();
    k_msleep(2000);
    
    test_gpio1_gpio2();
    k_msleep(2000);
    
    test_gpio3();
    k_msleep(2000);
    
    test_vout1_enable();
    k_msleep(2000);
    
    test_ls1();
    k_msleep(2000);
    
    test_ls2();
    k_msleep(2000);
    
    printk("\n=== All Tests Complete ===\n");
    
    /* After all tests, blink to show alive */
    while (1) {
        if (device_is_ready(leds)) {
            led_on(leds, 2U);  // Blue
            k_msleep(1000);
            led_off(leds, 2U);
            k_msleep(1000);
        }
    }
#endif
    
    /* If no test is defined, just blink */
    printk("\nNo test selected. Just blinking blue LED.\n");
    printk("Uncomment a test define in the code.\n");
    while (1) {
        if (device_is_ready(leds)) {
            led_on(leds, 2U);  // Blue
            k_msleep(1000);
            led_off(leds, 2U);
            k_msleep(1000);
        }
    }
    
    return 0;
}


// /*
//  * NPM1300 Simple Test Program - Quick Verification
//  * Updated for nrf54l15 with proper GPIO control
//  * 
//  * This simplified version allows manual testing of each feature
//  * without button controls. Just uncomment the test you want to run.
//  */

// #include <stdlib.h>
// #include <zephyr/kernel.h>
// #include <zephyr/device.h>
// #include <zephyr/drivers/gpio.h>
// #include <zephyr/drivers/led.h>
// #include <zephyr/drivers/mfd/npm1300.h>
// #include <zephyr/drivers/regulator.h>
// #include <zephyr/drivers/sensor.h>
// #include <zephyr/sys/printk.h>

// #define TEST_DELAY_MS 2000

// /* Device handles */
// static const struct device *pmic = DEVICE_DT_GET(DT_NODELABEL(npm1300_ek_pmic));
// static const struct device *leds = DEVICE_DT_GET(DT_NODELABEL(npm1300_ek_leds));
// static const struct device *regulators = DEVICE_DT_GET(DT_NODELABEL(npm1300_ek_regulators));

// /* GPIO controller - this is the npm1300 GPIO controller */
// static const struct device *npm_gpio = DEVICE_DT_GET(DT_NODELABEL(npm1300_ek_gpio));

// /* For load switches */
// static const struct device *ldsw1 = DEVICE_DT_GET(DT_NODELABEL(npm1300_ek_ldo1));
// static const struct device *ldsw2 = DEVICE_DT_GET(DT_NODELABEL(npm1300_ek_ldo2));

// /* For VOUT1 */
// static const struct device *vout1 = DEVICE_DT_GET(DT_NODELABEL(npm1300_ek_buck1));

// /* Test selection - uncomment ONE test at a time */
// // #define TEST_RGB_LEDS_ONLY    // Only RGB LEDs blink
// #define TEST_ALL_TOGETHER     // LEDs + GPIO1,2,3 + LS1,2 all toggle together
// // #define TEST_VOUT1            // VOUT1 enable test
// // #define TEST_ALL_SEQUENTIAL   // Run all tests one after another

// void test_rgb_leds(void)
// {
//     printk("\n=== RGB LED Test ===\n");
    
//     if (!device_is_ready(leds)) {
//         printk("ERROR: LED device not ready\n");
//         return;
//     }
    
//     for (int i = 0; i < 5; i++) {
//         printk("Cycle %d: LEDs ON\n", i + 1);
//         led_on(leds, 0U);  // Red
//         led_on(leds, 1U);  // Green
//         led_on(leds, 2U);  // Blue
//         k_msleep(TEST_DELAY_MS);
        
//         printk("Cycle %d: LEDs OFF\n", i + 1);
//         led_off(leds, 0U);
//         led_off(leds, 1U);
//         led_off(leds, 2U);
//         k_msleep(TEST_DELAY_MS);
//     }
    
//     printk("✓ RGB LED test complete\n");
// }

// void test_all_together(void)
// {
//     printk("\n=== ALL TOGETHER Test ===\n");
//     printk("RGB LEDs + GPIO1,2,3 + LS1,2 toggling synchronously\n");
//     printk("Measure: GPIO1, GPIO2, GPIO3 pins and PVDD1, PVDD2 voltages\n\n");
    
//     /* Check all devices */
//     if (!device_is_ready(leds)) {
//         printk("ERROR: LED device not ready\n");
//         return;
//     }
    
//     if (!device_is_ready(npm_gpio)) {
//         printk("ERROR: npm1300 GPIO controller not ready\n");
//         return;
//     }
    
//     if (!device_is_ready(ldsw1)) {
//         printk("ERROR: LS1 device not ready\n");
//         return;
//     }
    
//     if (!device_is_ready(ldsw2)) {
//         printk("ERROR: LS2 device not ready\n");
//         return;
//     }
    
//     /* Configure GPIOs as outputs */
//     int ret;
//     ret = gpio_pin_configure(npm_gpio, 1, GPIO_OUTPUT_INACTIVE);
//     if (ret < 0) {
//         printk("ERROR: Failed to configure GPIO1 (error: %d)\n", ret);
//         return;
//     }
    
//     ret = gpio_pin_configure(npm_gpio, 2, GPIO_OUTPUT_INACTIVE);
//     if (ret < 0) {
//         printk("ERROR: Failed to configure GPIO2 (error: %d)\n", ret);
//         return;
//     }
    
//     ret = gpio_pin_configure(npm_gpio, 3, GPIO_OUTPUT_INACTIVE);
//     if (ret < 0) {
//         printk("ERROR: Failed to configure GPIO3 (error: %d)\n", ret);
//         return;
//     }
    
//     printk("✓ All devices configured successfully\n\n");
    
//     /* Toggle all together in sync */
//     for (int i = 0; i < 5; i++) {
//         printk("Cycle %d: ALL ON (LEDs + GPIO1,2,3 + LS1,2)\n", i + 1);
        
//         /* Turn on LEDs */
//         led_on(leds, 0U);  // Red
//         led_on(leds, 1U);  // Green
//         led_on(leds, 2U);  // Blue
        
//         /* Set GPIOs HIGH */
//         gpio_pin_set(npm_gpio, 1, 1);
//         gpio_pin_set(npm_gpio, 2, 1);
//         gpio_pin_set(npm_gpio, 3, 1);
        
//         /* Enable Load Switches */
//         regulator_enable(ldsw1);
//         regulator_enable(ldsw2);
        
//         k_msleep(TEST_DELAY_MS);
        
//         printk("Cycle %d: ALL OFF\n", i + 1);
        
//         /* Turn off LEDs */
//         led_off(leds, 0U);
//         led_off(leds, 1U);
//         led_off(leds, 2U);
        
//         /* Set GPIOs LOW */
//         gpio_pin_set(npm_gpio, 1, 0);
//         gpio_pin_set(npm_gpio, 2, 0);
//         gpio_pin_set(npm_gpio, 3, 0);
        
//         /* Disable Load Switches */
//         regulator_disable(ldsw1);
//         regulator_disable(ldsw2);
        
//         k_msleep(TEST_DELAY_MS);
//     }
    
//     printk("✓ ALL TOGETHER test complete\n");
// }

// void test_gpio1_gpio2(void)
// {
//     printk("\n=== GPIO1 & GPIO2 Test ===\n");
//     printk("Measure voltage at GPIO1 and GPIO2 pins\n");
    
//     if (!device_is_ready(npm_gpio)) {
//         printk("ERROR: npm1300 GPIO controller not ready\n");
//         return;
//     }
    
//     /* Configure GPIO1 (pin 1) and GPIO2 (pin 2) as outputs */
//     int ret;
//     ret = gpio_pin_configure(npm_gpio, 1, GPIO_OUTPUT_INACTIVE);
//     if (ret < 0) {
//         printk("ERROR: Failed to configure GPIO1 (error: %d)\n", ret);
//         return;
//     }
    
//     ret = gpio_pin_configure(npm_gpio, 2, GPIO_OUTPUT_INACTIVE);
//     if (ret < 0) {
//         printk("ERROR: Failed to configure GPIO2 (error: %d)\n", ret);
//         return;
//     }
    
//     for (int i = 0; i < 5; i++) {
//         printk("Cycle %d: GPIO1 & GPIO2 HIGH\n", i + 1);
//         gpio_pin_set(npm_gpio, 1, 1);
//         gpio_pin_set(npm_gpio, 2, 1);
//         k_msleep(TEST_DELAY_MS);
        
//         printk("Cycle %d: GPIO1 & GPIO2 LOW\n", i + 1);
//         gpio_pin_set(npm_gpio, 1, 0);
//         gpio_pin_set(npm_gpio, 2, 0);
//         k_msleep(TEST_DELAY_MS);
//     }
    
//     printk("✓ GPIO1/GPIO2 test complete\n");
// }

// void test_gpio3(void)
// {
//     printk("\n=== GPIO3 Test ===\n");
//     printk("Measure: EN_BIO test point & V_BIO at C19\n");
    
//     if (!device_is_ready(npm_gpio)) {
//         printk("ERROR: npm1300 GPIO controller not ready\n");
//         return;
//     }
    
//     /* Configure GPIO3 (pin 3) as output */
//     int ret = gpio_pin_configure(npm_gpio, 3, GPIO_OUTPUT_INACTIVE);
//     if (ret < 0) {
//         printk("ERROR: Failed to configure GPIO3 (error: %d)\n", ret);
//         return;
//     }
    
//     for (int i = 0; i < 5; i++) {
//         printk("Cycle %d: GPIO3 HIGH\n", i + 1);
//         gpio_pin_set(npm_gpio, 3, 1);
//         k_msleep(TEST_DELAY_MS);
        
//         printk("Cycle %d: GPIO3 LOW\n", i + 1);
//         gpio_pin_set(npm_gpio, 3, 0);
//         k_msleep(TEST_DELAY_MS);
//     }
    
//     printk("✓ GPIO3 test complete\n");
// }

// void test_vout1_enable(void)
// {
//     printk("\n=== VOUT1 Enable Test ===\n");
//     printk("Measure: 3V at C15\n");
    
//     if (!device_is_ready(vout1)) {
//         printk("ERROR: VOUT1 device not ready\n");
//         return;
//     }
    
//     int ret = regulator_set_voltage(vout1, 3000000, 3000000);
//     if (ret < 0) {
//         printk("ERROR: Failed to set voltage (error: %d)\n", ret);
//         return;
//     }
    
//     ret = regulator_enable(vout1);
//     if (ret < 0) {
//         printk("ERROR: Failed to enable VOUT1 (error: %d)\n", ret);
//         return;
//     }
    
//     printk("✓ VOUT1 enabled at 3V - measure at C15\n");
//     printk("Holding for 5 seconds for measurement...\n");
//     k_msleep(5000);
//     printk("✓ VOUT1 test complete (still enabled)\n");
// }

// void test_ls1(void)
// {
//     printk("\n=== Load Switch 1 Test ===\n");
//     printk("Measure: Voltage at PVDD1\n");
    
//     if (!device_is_ready(ldsw1)) {
//         printk("ERROR: LS1 device not ready\n");
//         return;
//     }
    
//     for (int i = 0; i < 5; i++) {
//         printk("LS1 ENABLE\n");
//         regulator_enable(ldsw1);
//         k_msleep(TEST_DELAY_MS);
        
//         printk("LS1 DISABLE\n");
//         regulator_disable(ldsw1);
//         k_msleep(TEST_DELAY_MS);
//     }
    
//     printk("✓ LS1 test complete\n");
// }

// void test_ls2(void)
// {
//     printk("\n=== Load Switch 2 Test ===\n");
//     printk("Measure: Voltage at PVDD2\n");
    
//     if (!device_is_ready(ldsw2)) {
//         printk("ERROR: LS2 device not ready\n");
//         return;
//     }
    
//     for (int i = 0; i < 5; i++) {
//         printk("LS2 ENABLE\n");
//         regulator_enable(ldsw2);
//         k_msleep(TEST_DELAY_MS);
        
//         printk("LS2 DISABLE\n");
//         regulator_disable(ldsw2);
//         k_msleep(TEST_DELAY_MS);
//     }
    
//     printk("✓ LS2 test complete\n");
// }

// int main(void)
// {
//     printk("\n========================================\n");
//     printk("  NPM1300 Simple Test Program\n");
//     printk("  Version: 1.3 (Combined Tests)\n");
//     printk("========================================\n\n");
    
//     /* Test 1: I2C Communication */
//     printk("TEST 1: I2C Communication\n");
//     if (device_is_ready(pmic)) {
//         printk("✓ PMIC device ready - I2C OK\n");
//     } else {
//         printk("✗ PMIC device not ready - I2C FAILED\n");
//         return 0;
//     }
    
//     /* Check GPIO controller */
//     if (device_is_ready(npm_gpio)) {
//         printk("✓ npm1300 GPIO controller ready\n");
//     } else {
//         printk("⚠ npm1300 GPIO controller not ready (GPIO tests will fail)\n");
//     }
    
//     k_msleep(1000);
    
//     /* Run selected test(s) */
// #ifdef TEST_RGB_LEDS_ONLY
//     printk("\n=== Running RGB LED Test ONLY (Continuously) ===\n");
//     printk("Press reset to stop\n\n");
//     while (1) {
//         test_rgb_leds();
//         k_msleep(1000);
//     }
// #endif

// #ifdef TEST_ALL_TOGETHER
//     printk("\n=== Running ALL TOGETHER Test (Continuously) ===\n");
//     printk("LEDs + GPIO1,2,3 + LS1,2 toggling synchronously\n");
//     printk("Press reset to stop\n\n");
//     while (1) {
//         test_all_together();
//         k_msleep(1000);
//     }
// #endif

// #ifdef TEST_VOUT1
//     printk("\n=== Running VOUT1 Test Once ===\n");
//     printk("VOUT1 will remain enabled\n\n");
//     test_vout1_enable();
//     printk("\n=== VOUT1 Test Complete - Staying ON ===\n");
//     printk("Measure voltage at C15. Press reset to exit.\n");
//     /* Stay alive to keep VOUT1 enabled */
//     while (1) {
//         k_msleep(10000);
//     }
// #endif

// #ifdef TEST_ALL_SEQUENTIAL
//     printk("\n=== Running All Tests Sequentially ===\n");
//     k_msleep(1000);
    
//     test_rgb_leds();
//     k_msleep(2000);
    
//     test_gpio1_gpio2();
//     k_msleep(2000);
    
//     test_gpio3();
//     k_msleep(2000);
    
//     test_vout1_enable();
//     k_msleep(2000);
    
//     test_ls1();
//     k_msleep(2000);
    
//     test_ls2();
//     k_msleep(2000);
    
//     printk("\n=== All Tests Complete ===\n");
    
//     /* After all tests, blink to show alive */
//     while (1) {
//         if (device_is_ready(leds)) {
//             led_on(leds, 2U);  // Blue
//             k_msleep(1000);
//             led_off(leds, 2U);
//             k_msleep(1000);
//         }
//     }
// #endif
    
//     /* If no test is defined, just blink */
//     printk("\nNo test selected. Just blinking blue LED.\n");
//     printk("Uncomment a test define in the code.\n");
//     while (1) {
//         if (device_is_ready(leds)) {
//             led_on(leds, 2U);  // Blue
//             k_msleep(1000);
//             led_off(leds, 2U);
//             k_msleep(1000);
//         }
//     }
    
//     return 0;
// }