#ifndef BTSTACK_CONFIG_H
#define BTSTACK_CONFIG_H

// Basic minimal configuration
#define ENABLE_LOG_INFO
#define ENABLE_LOG_ERROR

// Pico W specific
#define ENABLE_LE_PERIPHERAL
#define MAX_NR_LE_DEVICE_DB_ENTRIES 10
#define HCI_ACL_PAYLOAD_SIZE 1691

#endif