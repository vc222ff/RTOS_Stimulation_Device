// Imports project dependencies.
#include <FreeRTOS.h>
#include <task.h>
#include <stdint.h>
#include <stdio.h>
#include <hardware/i2c.h>
#include <pico/stdlib.h>
#include <pico/cyw43_arch.h>
#include <pico/btstack_cyw43.h>
#include <btstack.h>

// Imports BLE server header file.
#include "ble_server.h"


// Declares preprocessor macros for settings. (Before compilation).
#define VIBRATION_MOTOR_VCC 21          // Vibration motor GPIO pin.
#define IMU_UPPER_VCC 14                // Power (VCC) pin for upper sensor.
#define IMU_LOWER_VCC 1                 // Power pin for lower sensor.

#define IMU_UPPER_SCL 13                // SCL GPIO connection for upper sensor.
#define IMU_UPPER_SDA 12                // SDA GPIO connection for upper sensor. 
#define IMU_LOWER_SCL 3                 // SCL GPIO connection for lower sensor.
#define IMU_LOWER_SDA 2                 // SDA GPIO connection for lower sensor.

#define IMU_UPPER_I2C_BUS i2c0          // I2C communication bus for upper sensor.
#define IMU_LOWER_I2C_BUS i2c1          // I2C comminication bus for lower sensor.
#define I2C_CLOCK_SPEED 400000          // I2C communication bus speed.

#define MPU_6250_ADDRESS 0x68           // Standard memory adress for MPU_6250 sensors.
#define ACCEL_REG 0x3B                  // Start register for MPU accelerometer data (6 bytes).
#define PWR_MGMT_REG 0x6b               // MPU_6250 Power management register.


// Global variables.
static btstack_timer_source_t heartbeat;
static btstack_packet_callback_registration_t hci_event_callback_registration;


// ____ ____ .      // ORIGINALLY (heartbeat_handler() in server.c)
static void heartbeat_handler(struct btstack_timer_source *ts) {
    static uint32_t counter = 0;
    counter++;

    // Update the "TEMP" every 3 seconds
    if (counter % 3 == 0) {
        float deg_c = 27.0f - 16384.0f;
        current_temp = (uint16_t) 27;
        printf("Write temp %.2f degc\n", deg_c);
        if (le_notification_enabled) {
            att_server_request_can_send_now_event(con_handle);
        }
    }

    // Invert the LED
    static int led_on = true;
    led_on = !led_on;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);

    // Restart timer
    btstack_run_loop_set_timer(ts, 1000);
    btstack_run_loop_add_timer(ts);
}


                        
// Initializes Bluetooth Low Energy, BLE ___  .     // ORIGINALLY (main() in server.c)
void init_ble() {
    
    // Initializes
    cyw43_arch_init();
    l2cap_init();
    sm_init();


    att_server_init(profile_data, att_read_callback, att_write_callback);

    // inform about BTstack state
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // register for ATT event
    att_server_register_packet_handler(packet_handler);

    // setup heartbeat
    heartbeat.process = &heartbeat_handler;
    btstack_run_loop_set_timer(&heartbeat, 1000);
    btstack_run_loop_add_timer(&heartbeat);

    // turn on bluetooth!
    hci_power_control(HCI_POWER_ON);
}


// Initiates pins, I2C communication and power for MPU_6250 sensor.
void init_mpu(uint8_t SCL, uint8_t SDA, uint8_t VCC, uint8_t addr, i2c_inst_t *bus) {
    
    // Sets pins to function as I2C communication pins.
    gpio_set_function(SCL, GPIO_FUNC_I2C);
    gpio_set_function(SDA, GPIO_FUNC_I2C);
    
    // Enables built-in pull-up resistors (50 kΩ) on pins. 
    gpio_pull_up(SCL);
    gpio_pull_up(SDA);

    // Enables 3.3V current to the power VCC pin.
    gpio_init(VCC);
    gpio_set_dir(VCC, GPIO_OUT);
    gpio_put(VCC, 1);

    // Writes 0x00 data to PWR_MGNT_REG to turn on sensor device.
    uint8_t buf[2] = {PWR_MGMT_REG, 0x00};
    i2c_write_blocking(bus, addr, buf, 2, false);
}


// Retrieves accelerometer I2C data from MPU_6250 sensor.
void read_accelerometer(uint8_t addr, i2c_inst_t *bus, int16_t *ax, int16_t *ay, int16_t *az) {
    
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


// Enables vibration motor for haptic feedback.
void vibrate_motor(uint8_t PIN) {
    
    // Starts vibrating.
    gpio_put(PIN, 1);

    // Vibration pulse duration.
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Stops vibrating.
    gpio_put(PIN, 0);
}


// Converts float values to force measured in g (9.82 ms^2).
float convert_to_g(int16_t raw) {
    return (float)raw / 16384.0f;
}


// The main posture correction task run in FreeRTOS.
void posture_monitor_task(void *pvParameters) {

    // Defines vibration cooldown variables.
    static TickType_t last_vibration_time = 0;
    const TickType_t vibration_cooldown = pdMS_TO_TICKS(2000);  // 2s cooldown

    // Force threshold measured in g-forces.
    const int16_t posture_threshold = 0.3;
    
    // 
    while(1) {

        // Instances coordinate variables.
        int16_t ax1, ay1, az1, ax2, ay2, az2;

        // Retrieves readings from both sensors.
        read_accelerometer(MPU_6250_ADDRESS, IMU_UPPER_I2C_BUS, &ax1, &ay1, &az1);
        read_accelerometer(MPU_6250_ADDRESS, IMU_LOWER_I2C_BUS, &ax2, &ay2, &az2);

        // Converts float values to g-forces.
        float ax1_g = convert_to_g(ax1);
        float ay1_g = convert_to_g(ay1);
        float az1_g = convert_to_g(az1);
        float ax2_g = convert_to_g(ax2);
        float ay2_g = convert_to_g(ay2);
        float az2_g = convert_to_g(az2);

        // Prints results.
        printf("MPU1: X=%.2f g, Y=%.2f g, Z=%.2f g\n", ax1_g, ay1_g, az1_g);
        printf("MPU2: X=%.2f g, Y=%.2f g, Z=%.2f g\n", ax2_g, ay2_g, az2_g);


        // Checks if posture is above the threshold.
        if ((ax1_g > posture_threshold || ay1_g > posture_threshold ||
            ax2_g > posture_threshold || ay2_g > posture_threshold) &&
            (xTaskGetTickCount() - last_vibration_time > vibration_cooldown)) {

            // Vibrates the motors
            printf("Warning: Bad Posture Detected!\n");
            vibrate_motor(VIBRATION_MOTOR_VCC);

            // Updates last vibration time
            last_vibration_time = xTaskGetTickCount();

        } else {

            printf("Posture OK\n");
        }

        // Adds a delay to reading
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}


// Entry point program.
int main() {
    
    // Initializes the standard C library.
    stdio_init_all();
    
    // Initializes BLE advertisement and related wireless protocols.  
    init_ble();

    // Iniitalizes the I2C communication buses.
    i2c_init(IMU_UPPER_I2C_BUS, I2C_CLOCK_SPEED);
    i2c_init(IMU_LOWER_I2C_BUS, I2C_CLOCK_SPEED);
    
    // Timer before Auxiliaries (IO) start.
    sleep_ms(3000);
    
    // Initalizes the MPU_6250 sensors. 
    init_mpu(IMU_UPPER_SCL, IMU_UPPER_SDA, IMU_UPPER_VCC, MPU_6250_ADDRESS, IMU_UPPER_I2C_BUS);
    init_mpu(IMU_LOWER_SCL, IMU_LOWER_SDA, IMU_LOWER_VCC, MPU_6250_ADDRESS, IMU_LOWER_I2C_BUS);

    // Initalizes the vibration motor. 
    init_motor(VIBRATION_MOTOR_VCC);

    // Creates a FreeRTOS task for posture monitoring. 
    xTaskCreate(posture_monitor_task, "PostureMonitor", 1024, NULL, 1, NULL);

    // Start FreeRTOS scheduler
    vTaskStartScheduler();

    // Unreachable statement.
    while(1) {}
}
