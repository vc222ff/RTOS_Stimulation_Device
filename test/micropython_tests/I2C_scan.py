from machine import Pin, I2C

# Configure GPIO15 as output
led = Pin(17, Pin.OUT)

# Set it HIGH (3.3V)
led.value(1)

# Initialize I2C (using GP pins for SDA and SCL)
i2c = I2C(1, scl=Pin(19), sda=Pin(18), freq=100000) # 400kHz or 100 kHz

# Scan I2C bus for connected devices
devices = i2c.scan()

if devices:
    print("I2C devices found at addresses:", [hex(dev) for dev in devices])
else:
    print("No I2C devices found. Check connections!")

