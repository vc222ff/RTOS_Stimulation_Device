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

// Imports flash storage header file.
#include "flash_storage.h"

// Declares preprocessor macros for settings. (Before compilation).
#define VIBRATION_MOTOR_VCC 21          // Vibration motor GPIO pin.
#define IMU_UPPER_VCC 14                // Power (VCC) pin for upper sensor.
#define IMU_LOWER_VCC 1                 // Power pin for lower sensor.

#define IMU_UPPER_SDA 12                // SDA GPIO connection for upper sensor. 
#define IMU_UPPER_SCL 13                // SCL GPIO connection for upper sensor.
#define IMU_LOWER_SDA 2                 // SDA GPIO connection for lower sensor.
#define IMU_LOWER_SCL 3                 // SCL GPIO connection for lower sensor.

#define IMU_UPPER_I2C_BUS i2c0          // I2C communication bus for upper sensor.
#define IMU_LOWER_I2C_BUS i2c1          // I2C comminication bus for lower sensor.
#define I2C_CLOCK_SPEED 400000          // I2C communication bus speed.

#define MPU_6050_ADDRESS 0x68           // Standard memory adress for MPU_6050 sensors.
#define ACCEL_REG 0x3B                  // Start register for MPU accelerometer data (6 bytes).
#define GYRO_REG 0x43                   // Start register for MPU gyroscopic data (6 bytes).
#define TEMP_REG 0x41                   // Start register for MPU temperature data (2 bytes).
#define PWR_MGMT_REG 0x6b               // MPU_6050 Power management register.


// Global variables.
static int16_t acceleration_1[3], acceleration_2[3];                            // 16-bit signed integer arrays for acceleration values.
static int16_t gyroscope_1[3], gyroscope_2[3];                                  // 16-bit signed integer arrays for gyroscope values.
static int16_t temperature_1, temperature_2;                                    // 16-bit signed integer for temperature values.

static btstack_packet_callback_registration_t hci_event_callback_registration;  // Structure for handling HCI events.
static btstack_timer_source_t ble_notify_timer;                                 // BTstack software timer for notifications.
static const int ble_notify_delay = 3000;                                       // Delay between the notifications.


// Handles timed updating of BLE payload and advertisement.
static void ble_handler(struct btstack_timer_source *ts) {
   
    // Updates content of BLE data payload string.
    snprintf(data_payload, PAYLOAD_LENGTH, 
        "ax1: %d | ay1: %d | az1: %d\ngx1: %d | gy1: %d | gz1: %d\nt1: %.2f\n\n"
        "ax2: %d | ay2: %d | az2: %d\ngx2: %d | gy2: %d | gz2: %d\nt2: %.2f",
        acceleration_1[0], acceleration_1[1], acceleration_1[2], gyroscope_1[0], gyroscope_1[1], gyroscope_1[2], 
        (temperature_1/340.0) + 36.53,
        acceleration_2[0], acceleration_2[1], acceleration_2[2], gyroscope_2[0], gyroscope_2[1], gyroscope_2[2], 
        (temperature_2/340.0) + 36.53
    );

    // Checks if client has enabled BLE notifications.
    if (le_notification_enabled) {
        att_server_request_can_send_now_event(con_handle);
    }

    // Restarts BTstack timer for callback to the BLE handler.
    btstack_run_loop_set_timer(ts, ble_notify_delay);
    btstack_run_loop_add_timer(ts);
}

               
// Initializes Bluetooth Low Energy BLE with its related components & protocols.
static void init_ble() {
    
    // Initializes CYW43 WiFi/BL chip, L2cap protocol and BL Security Manager.
    cyw43_arch_init();
    l2cap_init();
    sm_init();

    // Initializes BLE Attribute Protocol ATT server with callbacks.
    att_server_init(profile_data, att_read_callback, att_write_callback);

    // Sets callback and handler for HCI events.
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // Sets callback and handler for ATT packets.
    att_server_register_packet_handler(packet_handler);

    // Sets up periodic timer for BLE callback.
    ble_notify_timer.process = &ble_handler;
    btstack_run_loop_set_timer(&ble_notify_timer, ble_notify_delay);
    btstack_run_loop_add_timer(&ble_notify_timer);

    // Powers on the bluetooth stack.
    hci_power_control(HCI_POWER_ON);
}


// Initiates pins, I2C communication and power for MPU_6050 sensor.
static void init_mpu(i2c_inst_t *bus, uint8_t addr, uint8_t SCL, uint8_t SDA, uint8_t VCC) {
    
    // Sets pins to function as I2C communication pins.
    gpio_set_function(SDA, GPIO_FUNC_I2C);
    gpio_set_function(SCL, GPIO_FUNC_I2C);
    
    // Enables built-in pull-up resistors (50 kΩ) on pins. 
    gpio_pull_up(SDA);
    gpio_pull_up(SCL);
    
    // Enables 3.3V current to the power VCC pin.
    gpio_init(VCC);
    gpio_set_dir(VCC, GPIO_OUT);
    gpio_put(VCC, 1);

    // Writes 0x80 data to PWR_MGNT_REG register to reset device. 
    uint8_t buf[] = {PWR_MGMT_REG, 0x80};
    i2c_write_blocking(bus, addr, buf, 2, false);
    sleep_ms(100);

    // Writes 0x00 data to PWR_MGNT_REG register to turn on sensor device.
    buf[1] = 0x00;
    i2c_write_blocking(bus, addr, buf, 2, false);
    sleep_ms(10);
}


// Retrieves accelerometer I2C data from MPU_6050 sensor.
static void read_accelerometer(i2c_inst_t *bus ,uint8_t addr, int16_t accel[3]) {
    
    // Instances a register variable with memory location.
    uint8_t reg = ACCEL_REG;
    
    // Instances a 6-byte buffer for accelerometer readings.
    uint8_t buffer[6];

    // Requests and reads 6 bytes from acceelerometer register.
    i2c_write_blocking(bus, addr, &reg, 1, true);
    i2c_read_blocking(bus, addr, buffer , 6, false);

    // Reads 2 bytes 3 times and writes acceleration values to array.
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }
}


// Retrieves gyroscopic I2C data from MPU_6050 sensor.
static void read_gyroscope(i2c_inst_t *bus ,uint8_t addr, int16_t gyro[3]) {
    
    // Instances a register variable with memory location.
    uint8_t reg = GYRO_REG;
    
    // Instances a 6-byte buffer for gyroscopic readings.
    uint8_t buffer[6];

    // Requests and reads 6 bytes from gyroscope register.
    i2c_write_blocking(bus, addr, &reg, 1, true);
    i2c_read_blocking(bus, addr, buffer , 6, false);

    // Reads 2 bytes 3 times and writes gyroscopic values to array.
    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }
}


// Retrieves temperature I2C data from MPU_6050 sensor.
static void read_temperature(i2c_inst_t *bus ,uint8_t addr, int16_t *temp) {
    
    // Instances a register variable with memory location.
    uint8_t reg = TEMP_REG;
    
    // Instances a 2-byte buffer for temperature readings.
    uint8_t buffer[2];

    // Requests and reads 6 bytes from temperature register.
    i2c_write_blocking(bus, addr, &reg, 1, true);
    i2c_read_blocking(bus, addr, buffer , 2, false);

    // Reads 2 bytes and writes temperature to variable.
    *temp = buffer[0] << 8 | buffer[1];
}


// Initiates GPIO pin for vibration motor.
static void init_motor(uint8_t PIN) {

    // Initates pin and sets direction to output.
    gpio_init(PIN);
    gpio_set_dir(PIN, GPIO_OUT);
}


// Enables vibration motor for haptic feedback.
static void vibrate_motor(uint8_t PIN) {
    
    // Starts vibrating.
    gpio_put(PIN, 1);

    // Vibration pulse duration.
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Stops vibrating.
    gpio_put(PIN, 0);
}


// Converts float values to force measured in g (9.82 ms^2).
//float convert_to_g(int16_t raw) {
//    return (float)raw / 16384.0f;
//}


// The main posture correction task run in FreeRTOS.
static void posture_monitor_task(void *pvParameters) {

    // Defines vibration cooldown variables.
    static TickType_t last_vibration_time = 0;
    const TickType_t vibration_cooldown = pdMS_TO_TICKS(2000);  // 2s cooldown

    // Force threshold measured in g-forces.
    const int16_t posture_threshold = 0.3;
    
    // Boolean variable for toggling onboard LED.
    static int led_on = true;

    // While-true statement.
    while(1) {

        // Inverts the onboard LED to show processing.
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
        led_on = !led_on;

        // Retrieves accelerometer readings from both sensors.
        read_accelerometer(IMU_UPPER_I2C_BUS, MPU_6050_ADDRESS, acceleration_1);
        read_accelerometer(IMU_LOWER_I2C_BUS, MPU_6050_ADDRESS, acceleration_2);

        // Retrieves gyroscopic readings from both sensors.
        read_gyroscope(IMU_UPPER_I2C_BUS, MPU_6050_ADDRESS, gyroscope_1);
        read_gyroscope(IMU_LOWER_I2C_BUS, MPU_6050_ADDRESS, gyroscope_2);

        // Retrieves temperature readings from both sensors.
        read_temperature(IMU_UPPER_I2C_BUS, MPU_6050_ADDRESS, &temperature_1);
        read_temperature(IMU_LOWER_I2C_BUS, MPU_6050_ADDRESS, &temperature_2);

        // Vibrates the onboard motor.
        vibrate_motor(VIBRATION_MOTOR_VCC);

        
        // Prints results.
        //printf("MPU1: X=%.2f g, Y=%.2f g, Z=%.2f g\n", ax1_g, ay1_g, az1_g);
        //printf("MPU2: X=%.2f g, Y=%.2f g, Z=%.2f g\n", ax2_g, ay2_g, az2_g);

        // Checks if posture is above the threshold.
        //if ((ax1_g > posture_threshold || ay1_g > posture_threshold ||
        //    ax2_g > posture_threshold || ay2_g > posture_threshold) &&
        //    (xTaskGetTickCount() - last_vibration_time > vibration_cooldown)) {
        //
        //    // Vibrates the motors
        //    printf("Warning: Bad Posture Detected!\n");
        //    vibrate_motor(VIBRATION_MOTOR_VCC);
        //
        //    // Updates last vibration time
        //    last_vibration_time = xTaskGetTickCount();
        //
        //} else {
        //
        //    printf("Posture OK\n");
        //}

        // Adds a delay to reading.
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


// Entry point program.
static int main() {
    
    // Initializes the standard C library.
    stdio_init_all();
    
    // Initializes BLE advertisement and related wireless protocols.  
    init_ble();

    // Iniitalizes the I2C communication buses.
    i2c_init(IMU_UPPER_I2C_BUS, I2C_CLOCK_SPEED);
    i2c_init(IMU_LOWER_I2C_BUS, I2C_CLOCK_SPEED);
    
    // Timer before Auxiliaries (IO) start.
    sleep_ms(3000);
    
    // Initalizes the MPU_6050 sensors. 
    init_mpu(IMU_UPPER_I2C_BUS, MPU_6050_ADDRESS, IMU_UPPER_SCL, IMU_UPPER_SDA, IMU_UPPER_VCC);
    init_mpu(IMU_LOWER_I2C_BUS, MPU_6050_ADDRESS, IMU_LOWER_SCL, IMU_LOWER_SDA, IMU_LOWER_VCC);

    // Initalizes the vibration motor. 
    init_motor(VIBRATION_MOTOR_VCC);

    // Creates a FreeRTOS task for posture monitoring. 
    xTaskCreate(posture_monitor_task, "PostureMonitor", 1024, NULL, 1, NULL);

    // Start FreeRTOS scheduler
    vTaskStartScheduler();

    // Unreachable statement.
    while(1) {}
}
