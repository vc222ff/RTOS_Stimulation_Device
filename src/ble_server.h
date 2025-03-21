/**
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
*/

// Checks that ble_server.h header is not defined. Defines it.
#ifndef BLE_SERVER_H_
#define BLE_SERVER_H_

// Declares string buffer size for BLE message payload.
#define PAYLOAD_LENGTH 256
 
// BLE client notification flag and HCI/BL connection handle.
extern int le_notification_enabled;
extern hci_con_handle_t con_handle;

// BLE data payload of UTF-8 encoded string and its length.
extern char data_payload[PAYLOAD_LENGTH];
extern uint8_t const profile_data[];

// Server.c functions.
void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
uint16_t att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size);
int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size);

#endif