from machine import Pin, I2C, SoftI2C

# Hardware I2C
hw_i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=100000)

# Software I2C
sw_i2c = SoftI2C(scl=Pin(5), sda=Pin(4), freq=100000) 

print("MicroPython Support")
print(dir(machine))

# Printouts
print("Hardware I2C scan result:", hw_i2c.scan())
print("Software I2C scan result:", sw_i2c.scan())



#from machine import Pin, I2C, SoftI2C

# Hardware I2C & Software I2C
#hw_i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=100000)
#sw_i2c = SoftI2C(scl=Pin(27), sda=Pin(26), freq=100000) 

#print("MicroPython Support")
#print(dir(machine))

#try:
#    hw_I2C_devices = hw_i2c.scan()
#    sw_I2C_devices = sw_i2c.scan()
#    if hw_I2C_devices or sw_I2C_devices:
#        print("Hardware I2C devices found at:", [hex(d) for d in hw_I2C_devices])
#        print("Software I2C devices found at:", [hex(d) for d in sw_I2C_devices])
#    else:
#        print("No I2C devices found using I2C!")
        
#except Exception as e:
#    print("I2C scan failed:", e)
