/**
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

 #ifndef BLE_SERVER_H_
 #define BLE_SERVER_H_

 // Declares string buffer size for BLE message.
 #define ACCEL_STR_LEN 100

 extern int le_notification_enabled;
 extern hci_con_handle_t con_handle;
 extern uint16_t current_temp;
 extern char accel_string[ACCEL_STR_LEN];     // UTF-8 encoded string of accelerometer readings.  
 extern uint8_t const profile_data[];
 
 void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
 uint16_t att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size);
 int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size);
 
 #endif