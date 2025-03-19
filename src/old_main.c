// Project dependencies
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"


// Global constants
#define POSTURE_THRESHOLD 0.3 // Threshold in g
#define VIBRATION_MOTOR_PIN1 7 // Vibration motor 1 GPIO
#define VIBRATION_MOTOR_PIN2 8 // Vibration motor 2 GPIO
#define I2C_PORT i2c0
#define MPU1_ADDR 0x76  // First MPU-9250 sensor  (AD0 = GND)
#define MPU2_ADDR 0x76  // Second MPU-9250 sensor (AD0 = 3.3V)
#define ACCEL_REG 0x3B  // Start register for accelerometer data (6 bytes)
#define PWR_MGMT_1 0x6b // Power management register


void init_mpu(uint8_t addr) {
    uint8_t buf[2] = {PWR_MGMT_1, 0x00}; // Write 0x00 to PWR_MGMT_1
    i2c_write_blocking(I2C_PORT, addr, buf, 2, false);
}


void init_motor(uint8_t PIN) {
    gpio_init(PIN);
    gpio_set_dir(PIN, GPIO_OUT);
}


void read_accelerometer(uint8_t addr, int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t buffer[6];

    // Request 6 bytes from accelerometer registers
    uint8_t reg = ACCEL_REG;
    i2c_write_blocking(I2C_PORT, addr, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, addr, buffer , 6, false);

    // Combine high and low bytes
    *ax = (buffer[0] << 8) | buffer[1]; 
    *ay = (buffer[2] << 8) | buffer[3];
    *az = (buffer[4] << 8) | buffer[5];
}


float convert_to_g(int16_t raw) {
    return (float)raw / 16384.0f;
}


void vibrate_motors(uint8_t PIN_1, uint8_t PIN_2) {
    // Start vibrating
    gpio_put(PIN_1, 1);
    gpio_put(PIN_2, 1);

    // Pulse duration
    vTaskDelay(pdMS_TO_TICKS(500));

    // Stop vibrating
    gpio_put(PIN_1, 0);
    gpio_put(PIN_2, 0);
}


void posture_monitor_task(void *pvParameters) {
    // Defines vibration cooldown variables
    static TickType_t last_vibration_time = 0;
    const TickType_t vibration_cooldown = pdMS_TO_TICKS(2000);  // 2s cooldown

    while(1) {
        // Instances location variables
        int16_t ax1, ay1, az1, ax2, ay2, az2;

        // Read both sensors
        read_accelerometer(MPU1_ADDR, &ax1, &ay1, &az1);
        read_accelerometer(MPU2_ADDR, &ax2, &ay2, &az2);

        // Convert to g-force
        float ax1_g = convert_to_g(ax1);
        float ay1_g = convert_to_g(ay1);
        float az1_g = convert_to_g(az1);
        float ax2_g = convert_to_g(ax2);
        float ay2_g = convert_to_g(ay2);
        float az2_g = convert_to_g(az2);

        // Print results
        printf("MPU1: X=%.2f g, Y=%.2f g, Z=%.2f g\n", ax1_g, ay1_g, az1_g);
        printf("MPU2: X=%.2f g, Y=%.2f g, Z=%.2f g\n", ax2_g, ay2_g, az2_g);


        // Checks if posture is above the threshold
        if ((ax1_g > POSTURE_THRESHOLD || ay1_g > POSTURE_THRESHOLD ||
            ax2_g > POSTURE_THRESHOLD || ay2_g > POSTURE_THRESHOLD) &&
            (xTaskGetTickCount() - last_vibration_time > vibration_cooldown)) {

            // Vibrates the motors
            printf("Warning: Bad Posture Detected!\n");
            vibrate_motors(VIBRATION_MOTOR_PIN1, VIBRATION_MOTOR_PIN2);

            // Updates last vibration time
            last_vibration_time = xTaskGetTickCount();
        } else {
            printf("Posture OK\n");
        }

        // Adds a delay for reading
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}


int main() {
    // Initialize the Pico SDK library
    stdio_init_all();

    // Initialize I2C ports 
    i2c_init(I2C_PORT, 400 * 1000);  // 400 kHz I2C clock speed
    gpio_set_function(2, GPIO_FUNC_I2C); // SDA (GP0)
    gpio_set_function(3, GPIO_FUNC_I2C); // SCL (GP1)
    gpio_pull_up(2);
    gpio_pull_up(3);

    // Initialize MPU-9250 sensors
    init_mpu(MPU1_ADDR);
    init_mpu(MPU2_ADDR);

    // Initialize the vibration motors
    init_motor(VIBRATION_MOTOR_PIN1);
    init_motor(VIBRATION_MOTOR_PIN2);

    // Create FreeRTOS task for posture monitoring
    xTaskCreate(posture_monitor_task, "PostureMonitor", 1024, NULL, 1, NULL);

    // Start FreeRTOS scheduler
    vTaskStartScheduler();

    while(1) {}
}
