// Checks that flash_storage.h header is not defined. Defines it.
#ifndef FLASH_STORAGE_H_
#define FLASH_STORAGE_H_

// Dependencies.
#include <stdint.h>                                       // Needed for int16_t, uint32_t.

// Declares preprocessor macros.
#define FLASH_SAVE_SPACE 4096                             // The size of the save in flash memory.
#define FLASH_SAVE_OFFSET (2048*1024 - FLASH_SAVE_SPACE)  // Declares offset in flash for storing calibration config. 
#define CALIBRATION_MAGIC 0xCA1B600D                      // Used to verify data validity. 

// Sensor calibration structure.
typedef struct {
    int16_t accelerometer_bias[3];
    int16_t gyroscope_bias[3];
} SensorCalibration;

// Full calibration config for two sensors.
typedef struct {
    SensorCalibration sensor_1;
    SensorCalibration sensor_2;
    uint32_t magic;
} CalibrationData;


// flash_storage.c functions.
void save_calibration_to_flash(CalibrationData *data);
CalibrationData read_calibration_from_flash();

#endif