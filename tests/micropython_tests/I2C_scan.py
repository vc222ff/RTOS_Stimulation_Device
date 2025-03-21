from machine import Pin, I2C

# Initialize I2C (using GP4 for SDA and GP5 for SCL)
i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=400000)

# Scan I2C bus for connected devices
devices = i2c.scan()

if devices:
    print("I2C devices found at addresses:", [hex(dev) for dev in devices])
else:
    print("No I2C devices found. Check connections!")

