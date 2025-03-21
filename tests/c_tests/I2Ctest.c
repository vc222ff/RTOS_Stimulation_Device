#include "pico/stdlib.h"
#include <stdio.h>


// Define I2C SCL pin (change this if using a different pin)
#define I2C_SCL_PIN 5  // Adjust according to your setup

void recover_i2c_bus() {
    // Configure SCL as GPIO output
    gpio_init(I2C_SCL_PIN);
    gpio_set_dir(I2C_SCL_PIN, GPIO_OUT);

    // Toggle SCL 10 times to clear any stuck device
    for (int i = 0; i < 10; i++) {
        gpio_put(I2C_SCL_PIN, 1);
        sleep_us(10);  // Small delay
        gpio_put(I2C_SCL_PIN, 0);
        sleep_us(10);
    }

    // Set SCL back to high before releasing control
    gpio_put(I2C_SCL_PIN, 1);
    sleep_us(10);

    // Reconfigure SCL for I2C operation (input mode)
    gpio_set_dir(I2C_SCL_PIN, GPIO_IN);
}

int main() {
    stdio_init_all();
    printf("Attempting I2C bus recovery...\n");

    recover_i2c_bus();

    printf("I2C bus recovery complete.\n");

    return 0;
}
