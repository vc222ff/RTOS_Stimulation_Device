    // Imports project dependencies.
    #include <FreeRTOS.h>
    #include <task.h>
    #include <stdint.h>
    #include <stdio.h>
    #include <hardware/i2c.h>
    #include <pico/stdlib.h>
    #include <pico/time.h>
    #include <pico/cyw43_arch.h>
    #include <pico/btstack_cyw43.h>
    #include <btstack.h>
    #include <math.h>

    // Imports project header files.
    #include "ble_server.h"                 // Imports BLE server header file.
    #include "flash_storage.h"              // Imports flash storage header file.
    #include "evaluation.h"                 // Imports evaluation header file.


    // Declares preprocessor macros for settings. (Before compilation).
    #define VIBRATION_MOTOR_VCC 21          // Vibration motor GPIO pin.
    #define INTERNAL_LED 11                 // Internal LED GPIO pin.           (PICO_DEFAULT_LED_PIN ?)
    #define IMU_UPPER_VCC 17                // Power (VCC) pin for upper sensor.     //14
    #define IMU_LOWER_VCC 14                // Power pin for lower sensor.           //1

    #define IMU_UPPER_SDA 18                // SDA GPIO connection for upper sensor. //12
    #define IMU_UPPER_SCL 19                // SCL GPIO connection for upper sensor. //13
    #define IMU_LOWER_SDA 12                // SDA GPIO connection for lower sensor. //2
    #define IMU_LOWER_SCL 13                // SCL GPIO connection for lower sensor. //3

    #define IMU_UPPER_I2C_BUS i2c1          // I2C communication bus for upper sensor. //i2c0
    #define IMU_LOWER_I2C_BUS i2c0          // I2C comminication bus for lower sensor. //i2c1
    #define I2C_CLOCK_SPEED 400000          // I2C communication bus speed.

    #define MPU_6050_ADDRESS 0x68           // Standard memory adress for MPU_6050 sensors.
    #define ACCEL_REG 0x3B                  // Start register for MPU accelerometer data (6 bytes).
    #define GYRO_REG 0x43                   // Start register for MPU gyroscopic data (6 bytes).
    #define TEMP_REG 0x41                   // Start register for MPU temperature data (2 bytes).
    #define PWR_MGMT_REG 0x6B               // MPU_6050 Power management register.
    #define WHO_AM_I_REG 0x75

    #define UPPER_ANGLE_THRESHOLD 15        // Upper pitch angle boundry threshold in degrees.
    #define LOWER_ANGLE_THRESHOLD 15        // Lower pitch angle boundry threshold in degrees.

    #define NUM_REFERENCE_SAMPLES 100       // Sample number used in error validation.

    // Global variables.
    static int16_t acceleration_1[3], acceleration_2[3];                            // 16-bit signed integer arrays for acceleration values.
    static int16_t gyroscope_1[3], gyroscope_2[3];                                  // 16-bit signed integer arrays for gyroscope values.
    static int16_t temperature_1, temperature_2;                                    // 16-bit signed integer for temperature values.
    static float g_forces_1[3], g_forces_2[3];                                      // Floating point values for acceleration in g (9.82 ms^2).
    static float filtered_gyro_1[3], filtered_accel_1[3];                           // Floating point values for kalman filtered acceleration.
    static float filtered_gyro_2[3], filtered_accel_2[3];                           // Floating point values for kalman filtered gyroscope values.
    static float kalman_gyro_q = 0.001f;                                            // Floating point value for kalman filter acceleration q-value.
    static float kalman_gyro_r = 0.03f;                                             // Floating point value for kalman filter acceleration r-value.
    static float kalman_accel_q = 0.001f;                                           // Floating point value for kalman filter gyroscope q-value.
    static float kalman_accel_r =  0.03f;                                           // Floating point value for kalman filter gyroscope r-value.
    static float comp_pitch_1 = 0, comp_pitch_2 = 0;                                // Floating point values for filtered sensor pitch angles.
    static float alpha = 0.95f;                                                     // Floating point value for alpha in complementary filter.

    static float baseline_pitch_1 = 0, baseline_pitch_2 = 0;                        // Floating point values for calibrated baseline pitch angles.

    static float gold_angles[NUM_REFERENCE_SAMPLES];                                // Floating point array for gold standard values.
    static float trunk_angles_sensors[NUM_REFERENCE_SAMPLES];                       // Floating point array for trunk angle values.
    static uint8_t sample_count = 0;                                                // 8-bit unsigned integer for iterating sample values.

    static btstack_packet_callback_registration_t hci_event_callback_registration;  // Structure for handling HCI events.
    static btstack_timer_source_t ble_notify_timer;                                 // BTstack software timer for notifications.
    static const int ble_notify_delay = 2000;                                       // Delay between the notifications.

    static CalibrationData calibration_config;                                      // Structure for calibration configuration from flash memory. 
    static EvaluationResult evaluation_result;                                      // Structure for error validation evaluation results.
    static KalmanSensor3D kalman_upper;                                             // Structure for the kalman filtering in upper sensor unit.
    static KalmanSensor3D kalman_lower;                                             // Structure for the kalman filtering in lower sensor unit.

    // External reference to BLE server response function.
    extern void send_ble_response(const char *res);


    // Forward declaration of BLE-request handler.
    void ble_request_handler(const char *req, uint16_t len);


    // Initiates pins, I2C communication and power for MPU_6050 sensor.
    static void init_mpu(i2c_inst_t *bus, uint8_t addr, uint8_t SCL, uint8_t SDA, uint8_t VCC) {
        
        // Sets pins to function as I2C communication pins.
        gpio_set_function(SDA, GPIO_FUNC_I2C);
        gpio_set_function(SCL, GPIO_FUNC_I2C);
        
        // Enables built-in pull-up resistors (50 kΩ) on pins. 
        //gpio_pull_up(SDA);
        //gpio_pull_up(SCL);
        
        // Enables 3.3V current to the power VCC pin.
        gpio_init(VCC);
        gpio_set_dir(VCC, GPIO_OUT);
        gpio_put(VCC, 1);
        sleep_ms(100);

        // Writes 0x80 data to PWR_MGNT_REG register to reset device. 
        uint8_t buf[] = {PWR_MGMT_REG, 0x80};
        i2c_write_blocking(bus, addr, buf, 2, false);
        sleep_ms(100);

        // Writes 0x00 data to PWR_MGNT_REG register to turn on sensor device.
        buf[1] = 0x00;
        i2c_write_blocking(bus, addr, buf, 2, false);
        sleep_ms(10);

        // Writes 0x01 data to PWR_MGNT_REG register to use gyro as clock source.
        buf[1] = 0x00;
        i2c_write_blocking(bus, addr, buf, 2, false);
        sleep_ms(10);

        // Sensor diagnostic check.
        uint8_t reg = WHO_AM_I_REG;
        uint8_t res;
        i2c_write_blocking(bus, addr, &reg, 1, true);
        i2c_read_blocking(bus, addr, &res, 1, false);
        printf("Sensor WHO_AM_I (0x%02X): 0x%02X\n", addr, res); // Expect "0x68".

        if (res != 0x68) {
            printf("ERROR: MPU-6050 at pins SDA: 0x%02X, SCL: 0x%02X did not respond correctly!\n", SDA, SCL);
        }
    }


    // Retrieves accelerometer I2C data from MPU_6050 sensor.
    static void read_accelerometer(i2c_inst_t *bus, uint8_t addr, int16_t accel[3], const int16_t *bias) {
        
        // Instances a register variable with memory location.
        uint8_t reg = ACCEL_REG;
        
        // Instances a 6-byte buffer for accelerometer readings.
        uint8_t buffer[6];

        // Requests and reads 6 bytes from acceelerometer register.
        i2c_write_blocking(bus, addr, &reg, 1, true);
        i2c_read_blocking(bus, addr, buffer , 6, false);

        // Reads 2 bytes 3 times and writes acceleration values to array.
        for (int i = 0; i < 3; i++) {
            int16_t raw = (buffer[i * 2] << 8) | buffer[i * 2 + 1];
            accel[i] = raw - (bias ? bias[i] : 0);
        }
    }


    // Retrieves gyroscopic I2C data from MPU_6050 sensor.
    static void read_gyroscope(i2c_inst_t *bus, uint8_t addr, int16_t gyro[3], const int16_t *bias) {
        
        // Instances a register variable with memory location.
        uint8_t reg = GYRO_REG;
        
        // Instances a 6-byte buffer for gyroscopic readings.
        uint8_t buffer[6];

        // Requests and reads 6 bytes from gyroscope register.
        i2c_write_blocking(bus, addr, &reg, 1, true);
        i2c_read_blocking(bus, addr, buffer , 6, false);

        // Reads 2 bytes 3 times and writes gyroscopic values to array.
        for (int i = 0; i < 3; i++) {
            int16_t raw = (buffer[i * 2] << 8) | buffer[i * 2 + 1];
            gyro[i] = raw - (bias ? bias[i] : 0);
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


    // Calibrates MPU_6050 sensor to mitigate built-in bias.
    static void calibrate_mpu(i2c_inst_t *bus ,uint8_t addr, SensorCalibration *sensor) {
        
        // Sample amount for calibration. 
        const int samples = 2000;

        // Arrays for raw sensor reading data.
        int16_t accel_raw[3], gyro_raw[3];
        
        // Arrays for sum of sensor pitch data.
        int32_t accel_sum[3] = {0};
        int32_t gyro_sum[3] = {0};

        // Collects multiple samples for more reliability.
        for (int i = 0; i < samples; i++) {

            // Retrieves sensor readings without bias.
            read_accelerometer(bus, addr, accel_raw, NULL);
            read_gyroscope(bus, addr, gyro_raw, NULL);

            // Sums up all sample values in one.
            for (int j = 0; j < 3; j++) {
                accel_sum[j] += accel_raw[j];
                gyro_sum[j] += gyro_raw[j];
            }
            
            // Small delay between readings.
            vTaskDelay(pdMS_TO_TICKS(10));  
        }
        
        // Stores the average bias values.
        for (int j = 0; j < 3; j++) {
            sensor->accelerometer_bias[j] = accel_sum[j] / samples;
            sensor->gyroscope_bias[j] = gyro_sum[j] / samples;
        }
    }


    // Initiates GPIO pin for output.
    static void init_output(uint8_t PIN) {

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


    // Retrieves or creates a new calibration config for flash_storage.
    static void check_calibration() {
        
        // Extracts calibration config from flash memory.
        calibration_config = read_calibration_from_flash();

        // Checks so that the calibration magic is valid.
        if (calibration_config.magic != CALIBRATION_MAGIC) {
            
            // Calibrates the MPU_6050 sensors.
            calibrate_mpu(IMU_UPPER_I2C_BUS, MPU_6050_ADDRESS, &calibration_config.sensor_1);
            calibrate_mpu(IMU_LOWER_I2C_BUS, MPU_6050_ADDRESS, &calibration_config.sensor_2);
            
            // Sets magic to ensure data validity. 
            calibration_config.magic = CALIBRATION_MAGIC;
            
            // Saves calibration config to flash memory.
            save_calibration_to_flash(&calibration_config);
        }
    }


    // Computes pitch angle in deg from accelerometer readings.
    float comp_pitch_angle(float ax, float ay, float az) {
        float angle = atan2f(ay, az) * (180.0f / M_PI);
        return angle + 90.0f;       // Offset upright position to 0°.
    }


    // Converts int-16t raw accelerometer data into g-forces (9.82ms^2).
    float comp_g_force(int16_t val) {
        return (float)val / 16384.0f;
    }


    // Calibrates personalized baseline pitch angles for posture detection logic.
    static void calibrate_baseline(i2c_inst_t *bus1 ,uint8_t addr1, i2c_inst_t *bus2 ,uint8_t addr2) {
        
        // Sample amount for calibration. 
        const int samples = 1000;               // 1000 * 10 = 10 000 ms (samples * sample rate = elapsed time).           

        // Arrays for sum of sensor pitch readings .
        int32_t pitch_upper = 0;
        int32_t pitch_lower = 0;

        // Gets the current absolute time.
        absolute_time_t last_time;
        last_time = get_absolute_time();

        // Collects multiple samples for more reliability.
        for (int i = 0; i < samples; i++) {

            // Retrieves sensor readings.
            read_accelerometer(bus1, addr1, acceleration_1, NULL);
            read_accelerometer(bus2, addr2, acceleration_2, NULL);
            read_gyroscope(bus1, addr1, gyroscope_1, NULL);
            read_gyroscope(bus2, addr2, gyroscope_2, NULL);
            
            // Converts raw readings to g-forces.
            g_forces_1[0] = comp_g_force(acceleration_1[1]);            // MPU-6050 sensor-axes. 2x (x,y,z).
            g_forces_1[1] = comp_g_force(-acceleration_1[0]);           // Here it is important to account for sensor missalignment.
            g_forces_1[2] = comp_g_force(acceleration_1[2]);            // (Important to remember to change w/ new design iteration!)
            g_forces_2[0] = comp_g_force(-acceleration_2[1]);                
            g_forces_2[1] = comp_g_force(acceleration_2[0]);            // This function could be changed to use arrays... ?!
            g_forces_2[2] = comp_g_force(acceleration_2[2]);            // (To reduce code duplication.)

            // Calculates time since last pitch angle computation. 
            absolute_time_t now = get_absolute_time();
            float dt = to_us_since_boot(now - last_time) / 1e6f;
            last_time = now;

            // Computes accelerometer pitch angle (deg) and gyro pitch rate (deg/s).
            float pitch_angle1 = comp_pitch_angle(g_forces_1[0], g_forces_1[1], g_forces_1[2]);
            float pitch_angle2 = comp_pitch_angle(g_forces_2[0], g_forces_2[1], g_forces_2[2]);
            float pitch_rate1 = gyroscope_1[1] / 131.0f;
            float pitch_rate2 = gyroscope_2[1] / 131.0f;

            // Applies Complementary filter to filter out sensor noise.
            comp_pitch_1 = alpha * (comp_pitch_1 + pitch_rate1 * dt) + (1.0f - alpha) * pitch_angle1;
            comp_pitch_2 = alpha * (comp_pitch_2 + pitch_rate2 * dt) + (1.0f - alpha) * pitch_angle2;

            // Adds pitch readings to sum of all values.
            pitch_upper += comp_pitch_1;
            pitch_lower += comp_pitch_2;

            // Small delay between readings.
            vTaskDelay(pdMS_TO_TICKS(10));              // Elapsed time = 10 000 ms = 10 s.
        }

        // Resets variable arrays for use in non-calibration readings.
        memset(acceleration_1, 0 , sizeof(acceleration_1));
        memset(acceleration_2, 0 , sizeof(acceleration_2));
        memset(gyroscope_1, 0 , sizeof(gyroscope_1));
        memset(gyroscope_2, 0 , sizeof(gyroscope_2));
        memset(g_forces_1, 0 , sizeof(g_forces_1));
        memset(g_forces_2, 0 , sizeof(g_forces_2));
        comp_pitch_1 = 0, comp_pitch_2 = 0;

        // Stores the calibrated baseline pitch values.
        baseline_pitch_1 = pitch_upper / samples;
        baseline_pitch_2 = pitch_lower / samples;
    }


    // Evaluates trunk angle against the reference standard.
    EvaluationResult evaluate_trunk_angle_accuracy() {
        
        EvaluationResult result = {0};
        if (sample_count == 0) return result;
        float sum_squared_error = 0.0f;

        for (int i = 0; i < sample_count; i++) {
            float error = trunk_angles_sensors[i] - gold_angles[i];
            sum_squared_error += error * error;
        }
        result.rmse = sqrtf(sum_squared_error / sample_count);    for (int i = 0; i < sample_count; i++) {
            if (trunk_angles_sensors[i] >= -15.0f && trunk_angles_sensors[i] <= 15.0f) {
                result.within_range++;
            }
        }
        result.total = sample_count;
        result.percent_within_range = (float)result.within_range / sample_count * 100.0f;    return result;
    }


    // Handles incoming requests to BLE server.
    void ble_request_handler(const char *req, uint16_t len) {
        
        // Handles "calibrate" cases.
        if (strncmp(req, "calibrate", len) == 0) {
            calibrate_baseline(IMU_UPPER_I2C_BUS, MPU_6050_ADDRESS, IMU_LOWER_I2C_BUS, MPU_6050_ADDRESS);
            send_ble_response("calibration:ok");
        } 
        // Handles "vibrate" cases.
        else if (strncmp(req, "vibrate", len) == 0) {
            vibrate_motor(VIBRATION_MOTOR_VCC);
            send_ble_response("vibrate:ok");
        }
        // Handles unknown/malformed cases.
        else {
            printf("Server received unknown BLE request: %.*s\n", len, req);
            send_ble_response("error:unknown");
        }
    }


    // Handles timed broadcasting of BLE payload and advertisement.
    static void ble_broadcast_handler(struct btstack_timer_source *ts) {
    
        // Updates content of BLE data payload string.
        snprintf(data_payload, PAYLOAD_LENGTH, 
            "X1: %.4f | Y1: %.4f | Z1: %.4f\nGX1: %d | GY1: %d | GZ1: %d\nT1: %.2f | Pitch1: %.2f°\n\n"
            "X2: %.4f | Y2: %.4f | Z2: %.4f\nGX2: %d | GY2: %d | GZ2: %d\nT2: %.2f | Pitch2: %.2f°",
            g_forces_1[0], g_forces_1[1], g_forces_1[2],
            gyroscope_1[0], gyroscope_1[1], gyroscope_1[2],
            (temperature_1/340.0) + 36.53, comp_pitch_1,
            g_forces_2[0], g_forces_2[1], g_forces_2[2],
            gyroscope_2[0], gyroscope_2[1], gyroscope_2[2],
            (temperature_2/340.0) + 36.53, comp_pitch_2
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
        
        // Initializes CYW43 BL chip, L2cap protocol and BL Security Manager.
        cyw43_arch_init_with_country(CYW43_COUNTRY_SWEDEN);
        l2cap_init();
        sm_init();

        // Restricts input/output capabilities in BL Security Manager.
        sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
        sm_set_authentication_requirements(SM_AUTHREQ_NO_BONDING);

        // Initializes BLE Attribute Protocol ATT server with callbacks.
        att_server_init(profile_data, att_read_callback, att_write_callback);

        // Sets callback and handler for HCI events.
        hci_event_callback_registration.callback = &packet_handler;
        hci_add_event_handler(&hci_event_callback_registration);

        // Sets callback and handler for ATT packets.
        att_server_register_packet_handler(packet_handler);

        // Sets up periodic timer for BLE callback.
        ble_notify_timer.process = &ble_broadcast_handler;
        btstack_run_loop_set_timer(&ble_notify_timer, ble_notify_delay);
        btstack_run_loop_add_timer(&ble_notify_timer);
        
        // Enables debug mode.
        //btstack_debug_enable(true);

        // Powers on the bluetooth stack.
        hci_power_control(HCI_POWER_ON);
    }


    // The main posture correction task run in FreeRTOS.
    static void posture_monitor_task(void *pvParameters) {
        
        // Defines vibration cooldown variables.
        static const TickType_t vibration_cooldown = pdMS_TO_TICKS(1000);  // 1s vibration cooldown
        static TickType_t last_vibration_time = 0;
        
        // Boolean variable for toggling onboard LED.
        static int led_on = true;
        
        // Thresholds for upper and lower spine pitch angles in degrees.
        const float angle_upper_threshold = UPPER_ANGLE_THRESHOLD;
        const float angle_lower_threshold = LOWER_ANGLE_THRESHOLD;
    
        // Gets the current absolute time.
        absolute_time_t last_time;
        last_time = get_absolute_time();

        // While-true statement.
        while(1) {
            
            // Retrieves accelerometer readings from both sensors.
            read_accelerometer(IMU_UPPER_I2C_BUS, MPU_6050_ADDRESS, acceleration_1, calibration_config.sensor_1.accelerometer_bias);
            read_accelerometer(IMU_LOWER_I2C_BUS, MPU_6050_ADDRESS, acceleration_2, calibration_config.sensor_2.accelerometer_bias);

            // Retrieves gyroscopic readings from both sensors.
            read_gyroscope(IMU_UPPER_I2C_BUS, MPU_6050_ADDRESS, gyroscope_1, calibration_config.sensor_1.gyroscope_bias);
            read_gyroscope(IMU_LOWER_I2C_BUS, MPU_6050_ADDRESS, gyroscope_2, calibration_config.sensor_2.gyroscope_bias);

            // Retrieves temperature readings from both sensors.
            read_temperature(IMU_UPPER_I2C_BUS, MPU_6050_ADDRESS, &temperature_1);
            read_temperature(IMU_LOWER_I2C_BUS, MPU_6050_ADDRESS, &temperature_2);
            
            // Filters out sensor noise from raw sensor readings through Kalman filter algorithm.
            for (int i = 0; i < 3; ++i) {
                filtered_accel_1[i] = kalman1d_update(&kalman_upper.accel[i], (float)acceleration_1[i]);
                filtered_gyro_1[i]  = kalman1d_update(&kalman_upper.gyro[i], (float)gyroscope_1[i]);
                filtered_accel_2[i] = kalman1d_update(&kalman_lower.accel[i], (float)acceleration_2[i]);
                filtered_gyro_2[i]  = kalman1d_update(&kalman_lower.gyro[i], (float)gyroscope_2[i]);
            }


            // Converts and sorts raw accelerometer readings into g-forces.
            g_forces_1[0] = comp_g_force(-acceleration_1[1]);           // MPU-6050 sensor-axes. 2x (x,y,z).
            g_forces_1[1] = comp_g_force(acceleration_1[0]);            // Here it is important to account for sensor missalignment.
            g_forces_1[2] = comp_g_force(acceleration_1[2]);            // (Important to remember to change w/ new design iteration!)
            g_forces_2[0] = comp_g_force(-acceleration_2[1]);
            g_forces_2[1] = comp_g_force(acceleration_2[0]);            // This function could be changed to use arrays... ?!
            g_forces_2[2] = comp_g_force(acceleration_2[2]);            // (To reduce code duplication...)
            
            // Calculates time since last pitch angle computation. 
            absolute_time_t now = get_absolute_time();
            float dt = to_us_since_boot(now - last_time) / 1e6f;
            last_time = now;
            
            // Computes accelerometer pitch angles in deg for both sensors.
            float pitch_angle_1 = comp_pitch_angle(g_forces_1[0], g_forces_1[1], g_forces_1[2]);
            float pitch_angle_2 = comp_pitch_angle(g_forces_2[0], g_forces_2[1], g_forces_2[2]);

            // Computes gyroscopic pitch rate in deg/s for both sensors.
            float pitch_rate_1 = gyroscope_1[1] / 131.0f;
            float pitch_rate_2 = gyroscope_2[1] / 131.0f;

            // Filters out sensor noise using Complementary filter.
            comp_pitch_1 = alpha * (comp_pitch_1 + pitch_rate_1 * dt) + (1.0f - alpha) * pitch_angle_1;
            comp_pitch_2 = alpha * (comp_pitch_2 + pitch_rate_2 * dt) + (1.0f - alpha) * pitch_angle_2;

            // Error validation computation.
            float trunk_angle = comp_pitch_1 - comp_pitch_2;
            if (sample_count < NUM_REFERENCE_SAMPLES) {
                trunk_angles_sensors[sample_count] = trunk_angle;
                gold_angles[sample_count] = 0.0f;
            }

            // Defines thresholds for bad upper and lower posture angles.
            bool bad_posture_threshold =
            fabsf(comp_pitch_1 - baseline_pitch_1) > angle_upper_threshold ||   // Uses absolute values (magnitude).
            fabsf(comp_pitch_2 - baseline_pitch_2) > angle_lower_threshold;

            // Checks if posture is outside of posture angle threshold or not.
            if (bad_posture_threshold && (xTaskGetTickCount() - last_vibration_time > vibration_cooldown)) {
                
                // Activates the vibration motor component. 
                printf("Warning: Bad Posture Detected!\n");
                vibrate_motor(VIBRATION_MOTOR_VCC);

                // Updates last vibration time.
                last_vibration_time = xTaskGetTickCount();
            }

            // Inverts the onboard LED to display processing state.
            static int blink_counter = 0;
            if (++blink_counter >= 30) {
                //gpio_put(INTERNAL_LED, led_on);
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);   // Pico 1 W
                led_on = !led_on;
                blink_counter = 0;
            }

            // Adds a delay to reading.
            vTaskDelay(pdMS_TO_TICKS(10));
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
        
        // Initalizes the MPU_6050 sensors. 
        init_mpu(IMU_UPPER_I2C_BUS, MPU_6050_ADDRESS, IMU_UPPER_SCL, IMU_UPPER_SDA, IMU_UPPER_VCC);
        init_mpu(IMU_LOWER_I2C_BUS, MPU_6050_ADDRESS, IMU_LOWER_SCL, IMU_LOWER_SDA, IMU_LOWER_VCC);

        // Initalizes the vibration motor.
        init_output(VIBRATION_MOTOR_VCC);

        // Performs the initial personalized calibration of sensors.
        calibrate_baseline(IMU_UPPER_I2C_BUS, MPU_6050_ADDRESS, IMU_LOWER_I2C_BUS, MPU_6050_ADDRESS);

        // Evaluates sensor measurements against reference standard values.
        evaluate_trunk_angle_accuracy();
        
        // Iniitalizes Kalman sensor filtering logic for both sensors.
        kalman3d_init(&kalman_upper, kalman_gyro_q, kalman_gyro_r, kalman_accel_q, kalman_accel_r);
        kalman3d_init(&kalman_lower, kalman_gyro_q, kalman_gyro_r, kalman_accel_q, kalman_accel_r);

        // Creates a FreeRTOS task for posture monitoring. 
        xTaskCreate(posture_monitor_task, "PostureMonitor", 1024, NULL, 1, NULL);

        // Start FreeRTOS scheduler
        vTaskStartScheduler();

        // Unreachable statement.
        while(1) {}
    }
