# Example code for PiicoDev Motion Sensor MPU6050
from PiicoDev_MPU6050 import PiicoDev_MPU6050
from PiicoDev_Unified import sleep_ms # Cross-platform compatible sleep function

# Bluetooth 
import bluetooth
from ble_uart_peripheral import BLEUART # Ensure this library is available

# General
from machine import Pin
from micropython import const
import time


# Function to send data over BLE
def send(data):
    """Send console output over Bluetooth UART as UTF-8"""
    for conn_handle in uart._connections:
        uart._ble.gatts_notify(conn_handle, uart._tx_handle, data.encode('utf-8'))

ble = bluetooth.BLE()
uart = BLEUART(ble, name="PicoBLE")
vibration_motor_vcc = machine.Pin(21, machine.Pin.OUT)
imu_upper_vcc = machine.Pin(14, machine.Pin.OUT)
imu_lower_vcc = machine.Pin(1, machine.Pin.OUT) 

print('\nStarting program')

# Sets sensor VCC power pins to HIGH
vibration_motor_vcc.value(0)
imu_upper_vcc.value(0)
imu_lower_vcc.value(0)
time.sleep(5)
imu_upper_vcc.value(1)
imu_lower_vcc.value(1)
time.sleep(3)

imu_upper = PiicoDev_MPU6050(0, 400_000, 12, 13)                    # def __init__(self, bus=None, freq=None, sda=None, scl=None, addr=_MPU6050_ADDRESS):
imu_lower = PiicoDev_MPU6050(1, 400_000, 2, 3)

print('\nStarting BLE advertisement!')

# Main loop - Sends logs every 5 seconds
counter = 0
while counter < 50:
    
    # Log message
    log_message = f"Log Entry {counter}: Pico BLE UART is working!"
    print(log_message)  # Print to USB serial
    
    # Accelerometer data
    accel_u = imu_upper.read_accel_data() # read the accelerometer [ms^-2]
    aX_u = accel_u["x"]
    aY_u = accel_u["y"]
    aZ_u = accel_u["z"]
    
    accel_l = imu_lower.read_accel_data() # read the accelerometer [ms^-2]
    aX_l = accel_l["x"]
    aY_l = accel_l["y"]
    aZ_l = accel_l["z"]
    acc_u_message = f"Upper MPU6250 readings - x1: {str(aX_u)}   y1: {str(aY_u)}   z1: {str(aZ_u)}"
    acc_l_message = f"Lower MPU6250 readings - x2: {str(aX_l)}   y2: {str(aY_l)}   z2: {str(aZ_l)}"
    
    print(acc_u_message + "\n" + acc_l_message)
    
    # Sends UART message
    send(log_message + "\n\n" + acc_u_message + "\n" + acc_l_message + "\n") # Send to Bluetooth UART
    
    # Pulses the vibration motor
    vibration_motor_vcc.value(1)
    time.sleep(1.2)
    vibration_motor_vcc.value(0)
    
    counter += 1
    time.sleep(5)
    
    # Gyroscope Data
#     gyro = motion.read_gyro_data()   # read the gyro [deg/s]
#     gX = gyro["x"]
#     gY = gyro["y"]
#     gZ = gyro["z"]
#     print("x:" + str(gX) + " y:" + str(gY) + " z:" + str(gZ))
    
    # Rough temperature
#     temp = motion.read_temperature()   # read the device temperature [degC]
#     print("Temperature: " + str(temp) + "°C")

    # G-Force
#     gforce = motion.read_accel_abs(g=True) # read the absolute acceleration magnitude
#     print("G-Force: " + str(gforce))

