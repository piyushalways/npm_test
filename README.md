# NPM1300 Test Suite

Comprehensive test program for npm1300 PMIC on nrf54l15dk.

## Features
- ✅ I2C Communication verification
- ✅ RGB LED control (GPIO0, GPIO1, GPIO2)
- ✅ GPIO1/GPIO2/GPIO3 direct control
- ✅ VOUT1 voltage regulation (3V test at C15)
- ✅ Load Switch 1 (LS1) toggle test at PVDD1
- ✅ Load Switch 2 (LS2) toggle test at PVDD2

## Quick Start

1. **Files needed:**
   - `npm1300_simple_test_with_gpio.c` - Test code
   - `npm1300_ek_with_gpio.overlay` - Device tree overlay

2. **Build:**
   ```bash
   west build -b nrf54l15dk_nrf54l15_cpuapp -p
   west flash
   ```

3. **Select Test:**
   Edit the `#define` section in the code:
   ```c
   // Uncomment ONE test to run continuously:
   // #define TEST_RGB_LEDS
   // #define TEST_GPIO1_GPIO2
   // #define TEST_GPIO3
   // #define TEST_VOUT1
   // #define TEST_LS1
   // #define TEST_LS2
   #define TEST_ALL_SEQUENTIAL  // Default: runs all tests once
   ```

## Test Cases

| Test | Measurement Point | Expected Result |
|------|-------------------|-----------------|
| I2C | Console log | Device ready messages |
| RGB LEDs | Visual | All LEDs blink |
| GPIO1/GPIO2 | GPIO pins | Voltage toggles 0V ↔ HIGH |
| GPIO3 | EN_BIO, C19 | Voltage toggles 0V ↔ HIGH |
| VOUT1 | C15 | 3.0V DC |
| LS1 | PVDD1 | Voltage toggles |
| LS2 | PVDD2 | Voltage toggles |

## Behavior

- **Specific test defined:** Runs continuously in a loop
- **TEST_ALL_SEQUENTIAL:** Runs all tests once, then blinks blue LED
- **TEST_VOUT1:** Runs once, keeps voltage enabled for measurement

## Equipment Needed
- Multimeter (DMM) for DC voltage measurements
- Oscilloscope (optional) for timing verification
- Serial terminal (115200 baud)

## Notes
- npm1300 GPIO controller is defined as child node in PMIC device tree
- GPIO pins 0-4 available (5 total GPIOs)
- Press reset to stop continuous tests
