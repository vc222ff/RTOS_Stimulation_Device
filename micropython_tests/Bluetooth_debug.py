from machine import Pin
import bluetooth
from micropython import const
from ble_uart_peripheral import BLEUART # Ensure this library is available
import time

ble = bluetooth.BLE()
uart = BLEUART(ble, name="PicoBLE")
led = Pin(25, Pin.OUT)  # Internal LED is on Pin 25

# Function to send data over BLE
def send(data):
    """Send console output over Bluetooth UART as UTF-8"""
    for conn_handle in uart._connections:
        uart._ble.gatts_notify(conn_handle, uart._tx_handle, data.encode('utf-8'))

print('\nStarted BLE advertisement!')

# Main loop - Send logs every 5 seconds
counter = 0
while counter < 25:
    led.value(1)
    time.sleep(2.0)
    led.value(0)
    log_message = f"Log Entry {counter}: Pico BLE UART is working!"
    print(log_message)  # Print to USB serial
    send(log_message + "\n")  # Send to Bluetooth UART
    counter += 1
    time.sleep(5)

