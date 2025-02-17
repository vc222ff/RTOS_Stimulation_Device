// Project dependencies
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"


// Global constants
#define POSTURE_THRESHOLD 0.3 // Threshold in g
#define I2C_PORT i2c0
#define MPU1_ADDR 0x68  // First MPU-9250 sensor  (AD0 = GND)
#define MPU2_ADDR 0x69  // Second MPU-9250 sensor (AD0 = 3.3V)
#define ACCEL_REG 0x3B  // Start register for accelerometer data (6 bytes)
#define PWR_MGMT_1 0x6b // Power management register 


void init_mpu(uint8_t addr) {
    uint8_t buf[2] = {PWR_MGMT_1, 0x00};        // Write 0x00 to PWR_MGMT_1
    i2c_write_blocking(I2C_PORT, addr, buf, 2, false);
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


void posture_monitor_task(void *pvParameters) {
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


        // Checks if posture is below threshold
        if (ax1_g > POSTURE_THRESHOLD || ay1_g > POSTURE_THRESHOLD) {
            printf("Warning: Bad Posture Detected!\n");
            // Trigger vibration motors via GPIO pins 
        }

        // Adds a delay for reading
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}


int main() {
    stdio_init_all();

    // Initialize I2C
    i2c_init(I2C_PORT, 400 * 1000);  // 400kHz I2C speed
    gpio_set_function(0, GPIO_FUNC_I2C); // SDA (GP0)
    gpio_set_function(1, GPIO_FUNC_I2C); // SCL (GP1)
    gpio_pull_up(0);
    gpio_pull_up(1);

    // Initialize MPU-9250 sensors
    init_mpu(MPU1_ADDR);
    init_mpu(MPU2_ADDR);

    // Initialize vibrations motors GPIO
    //gpio_init(VIBRATION_MOTOR_PIN1);
    //gpio_init(VIBRATION_MOTOR_PIN2);

    // Create FreeRTOS task for posture monitoring
    xTaskCreate(posture_monitor_task, "PostureMonitor", 1024, NULL, 1, NULL);

    // Start FreeRTOS scheduler
    vTaskStartScheduler();

    while(1) {}
}
