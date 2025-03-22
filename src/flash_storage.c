// Imports dependencies.
#include <pico/stdlib.h>
#include <hardware/flash.h>
#include <hardware/sync.h>

// Imports header files.
#include "flash_storage.h"

// Declares preprocessor macros.
#define FLASH_SAVE_SPACE 4096                                   // The size of the save in flash memory.
#define FLASH_SAVE_OFFSET (2048*1024 - FLASH_SAVE_SPACE)        // Declares offset in flash for storing calibration config. 
#define CALIBRATION_MAGIC  0xCA1B600D                           // Used to verify data validity. 


// 
void save_calibration_to_flash(CalibrationData *data) {

    //
    uint32_t interrupts = save_and_disable_interrupts();

    // Erases memory location in flash (last 4k bytes).
    flash_range_erase(FLASH_SAVE_OFFSET, FLASH_SAVE_SPACE);

    //
    flash_range_program(FLASH_SAVE_OFFSET, (const uint8_t *)data, sizeof(CalibrationData));
    
    //
    restore_interrupts(interrupts);
}


//
CalibrationData read_calibration_from_flash() {
    
    // Retrieves calibration config from flash memory.
    const CalibrationData *flash_calib_data = (const CalibrationData *)(XIP_BASE + FLASH_SAVE_OFFSET);
    
    // Controls that magic in calibration config matches. 
    if (flash_calib_data->magic == CALIBRATION_MAGIC) {
        return *flash_calib_data;
    } else {
        CalibrationData empty = {0};
        return empty;
    }
}
