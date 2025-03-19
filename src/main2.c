// Project dependencies.
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"


// #define I2C_PORT i2c0
//#define POSTURE_THRESHOLD 0.3 // Threshold in g


#define VIBRATION_MOTOR_VCC 21  // Vibration motor GPIO pin.
#define IMU_UPPER_VCC 14        // Power (VCC) pin for upper sensor.
#define IMU_LOWER_VCC 1         // Power pin for lower sensor.

#define IMU_UPPER_SCL 13        // SCL GPIO connection for upper sensor.
#define IMU_UPPER_SDA 12        // SDA GPIO connection for upper sensor. 
#define IMU_LOWER_SCL 3         // SCL GPIO connection for lower sensor.
#define IMU_LOWER_SDA 2         // SDA GPIO connection for lower sensor.

#define IMU_UPPER_I2C_BUS i2c0  // I2C communication bus for upper sensor.
#define IMU_LOWER_I2C_BUS i2c1  // I2C comminication bus for lower sensor.
#define I2C_CLOCK_SPEED 400000  // I2C communication bus speed.

#define MPU_6050_ADDRESS 0x68   // Standard memory adress for MPU_6250 sensors.
#define ACCEL_REG 0x3B          // Start register for MPU accelerometer data (6 bytes).
#define PWR_MGMT_REG 0x6b       // MPU_6250 Power management register.


//
void init_mpu(uint8_t SCL, uint8_t SDA, uint8_t addr, uint8_t bus) {
    
    // 
    gpio_set_function(SCL, GPIO_FUNC_I2C);
    gpio_set_function(SDA, GPIO_FUNC_I2C);
    
    //
    gpio_pull_up(SCL);
    gpio_pull_up(SDA);

    // Writes 0x00 data to PWR_MGNT_REG to turn on sensor device.
    uint8_t buf[2] = {PWR_MGMT_REG, 0x00};
    i2c_write_blocking(bus, addr, buf, 2, false);
}


//
void read_accelerometer(uint8_t addr, uint8_t bus, int16_t *ax, int16_t *ay, int16_t *az) {
    
    // Instances a register variable with memory location.
    uint8_t reg = ACCEL_REG;
    
    // Instances a buffer for storing accelerometer readings.
    uint8_t buffer[6];

    // Requests and reads 6 bytes from acceelerometer register.
    i2c_write_blocking(bus, addr, &reg, 1, true);
    i2c_read_blocking(bus, addr, buffer , 6, false);

    // Combines reading high and low bytes.
    *ax = (buffer[0] << 8) | buffer[1]; 
    *ay = (buffer[2] << 8) | buffer[3];
    *az = (buffer[4] << 8) | buffer[5];
}


// Initiates GPIO pin for vibration motor.
void init_motor(uint8_t PIN) {

    // Initates pin and sets direction to output.
    gpio_init(PIN);
    gpio_set_dir(PIN, GPIO_OUT);
}


// 
void vibrate_motor(uint8_t PIN) {
    
    // Starts vibrating.
    gpio_put(PIN, 1);

    // Vibration pulse duration.
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Stops vibrating.
    gpio_put(PIN, 0);
}


// 
float convert_to_g(int16_t raw) {
    return (float)raw / 16384.0f;
}


//
void posture_monitor_task(void *pvParameters) {
    // Defines vibration cooldown variables.
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


// Entry point program.
int main() {

    // Initializes the Pico SDK library.
    stdio_init_all();

    // Iniitalizes the I2C communication buses.
    i2c_init(IMU_UPPER_I2C_BUS, I2C_CLOCK_SPEED);
    i2c_init(IMU_LOWER_I2C_BUS, I2C_CLOCK_SPEED);

    // Initalizes the MPU_6250 sensors. 


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
