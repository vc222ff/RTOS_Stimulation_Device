/**
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
*/

// Imports dependencies.
#include <stdio.h>
#include <btstack.h>

// Imports header files.
#include "posture_monitor.h"
#include "ble_server.h"

// Declares preprocessor macros.
#define APP_AD_FLAGS 0x06

// Declares handle definition for incoming communication.
# define ATT_HANDLE_CUSTOM_RX_VALUE 0x0025

// Global variables.
int le_notification_enabled;                                           // BLE client notification flag.
hci_con_handle_t con_handle;                                           // The HCI/BL connection handle.

char data_payload[PAYLOAD_LENGTH];                                     // Outgoing string array with data payload.
char response_payload[PAYLOAD_LENGTH];                                 // Outgoing string array with response to request.
static bool send_response_next = false;                                // Boolean for marking outgoing packet as a response. 

static uint8_t adv_data[] = {                                          // BLE advertisement payload:
    0x02, BLUETOOTH_DATA_TYPE_FLAGS, APP_AD_FLAGS,                                      // Flags: General Discoverable.
    0x17, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME,                                      // Device Name.                           // -||-
    0x03, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS,              // UUIDs services.
    0x1a, 0x18,                                                                         // -||-
};
static const uint8_t adv_data_len = sizeof(adv_data);                  // Advertisement data length.


// External reference to main.c BLE request handler function.
extern void ble_request_handler(const char *req, uint16_t len);


// Packet handler function.
void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(size);
    UNUSED(channel);
    bd_addr_t local_addr;

    // Aborts if packet is not of HCI event type.
    if (packet_type != HCI_EVENT_PACKET) return;

    // Event type.
    uint8_t event_type = hci_event_packet_get_type(packet);
    
    // Switch statement for BLE event cases.
    switch(event_type) {
        case BTSTACK_EVENT_STATE: {
            uint8_t state = btstack_event_state_get_state(packet);
            printf("[BT] Stack state changed: 0x%02x\n", state);
            if (state != HCI_STATE_WORKING) return;
            
            gap_local_bd_addr(local_addr);
            printf("[BT] BTstack up and running on %s\n", bd_addr_to_str(local_addr));
                            
            // Settings for BLE advertisements.
            uint16_t adv_int_min = 800;            // Min interval period.
            uint16_t adv_int_max = 800;            // Max interval period.
            uint8_t adv_type = 0;                  // Advertisement type (ADV_IND).
            bd_addr_t null_addr = {0};
            memset(null_addr, 0, 6);
            gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);
            assert(adv_data_len <= 31);            // BLE limitation.
            gap_advertisements_set_data(adv_data_len, (uint8_t*) adv_data);
            gap_advertisements_enable(1);
            printf("[BT] Advertising started\n");
            break;
        }
        
        case HCI_EVENT_LE_META: {
            uint8_t subevent = hci_event_le_meta_get_subevent_code(packet);
            if (subevent == HCI_SUBEVENT_LE_CONNECTION_COMPLETE) {
                const uint8_t *subevent_data = &packet[3];
                uint8_t status = subevent_data[0];
                hci_con_handle_t handle = little_endian_read_16(subevent_data, 1);
                printf("[BLE] LE Connection attempt. Status: 0x%02x, Handle: 0x%04x\n", status, handle);
                if (status == ERROR_CODE_SUCCESS) {
                    con_handle = handle;
                    printf("[BLE] Successfully connected! Handle: 0x%04x\n", con_handle);
                } else {
                    printf("[BLE] Connection failed. Status: 0x%02x\n", status);
                }
            } else {
                printf("[BLE] Unhandled LE Meta event: 0x%02x\n", subevent);
            }
            break;
        }
        

        case HCI_EVENT_DISCONNECTION_COMPLETE: {
            uint8_t reason = packet[5];
            printf("[BLE] Disconnected. Reason: 0x%02x\n", reason);
            le_notification_enabled = 0;
            break;
        }
            
        case ATT_EVENT_CAN_SEND_NOW: {
            printf("[BLE] ATT_EVENT_CAN_SEND_NOW triggered\n");
            if (send_response_next) {
                printf("[BLE] Sending queued response: %s\n", response_payload);
                att_server_notify(
                    con_handle, 
                    ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE_01_VALUE_HANDLE, 
                    (uint8_t*)response_payload, 
                    strlen(response_payload)
                );
                send_response_next = false;
            } else {
                printf("[BLE] Sending standard payload: %s\n", data_payload);
                att_server_notify(
                    con_handle, 
                    ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE_01_VALUE_HANDLE, 
                    (uint8_t*)data_payload, 
                    strlen(data_payload));
            }
            break;
        }

        default: {
            printf("[BT] Unhandled event type: 0x%02x\n", event_type);
            break;
        }
    }
}


// BLE read callback function.
uint16_t att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size) {
    UNUSED(connection_handle);

    if (att_handle == ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE_01_VALUE_HANDLE){
        return att_read_callback_handle_blob((const uint8_t *)data_payload, sizeof(data_payload), offset, buffer, buffer_size);
    }
    return 0;
}


// BLE write callback function. 
int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size) {
    UNUSED(transaction_mode);
    UNUSED(offset);
    
    if (att_handle == ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE_01_CLIENT_CONFIGURATION_HANDLE) {
        le_notification_enabled = little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
        if (le_notification_enabled) {
            att_server_request_can_send_now_event(connection_handle);
        }
        return 0;
    }

    // Handler for BLE RX Write operations.
    if (att_handle == ATT_CHARACTERISTIC_b17a58e4_3f6d_4ae0_a70a_1656a3e59be1_01_VALUE_HANDLE) {

        // Calls BLE request handler in main.c file.
        ble_request_handler((const char*)buffer, buffer_size);
        return 0;
    }

    return 0;
}


// BLE write callback function. 
void send_ble_response(const char *res) {
    if (!le_notification_enabled) return;

    snprintf(response_payload, PAYLOAD_LENGTH, "%s", res);
    send_response_next = true;
    att_server_request_can_send_now_event(con_handle);
}
