from time import sleep
from encoderDriver import *

# Encoder 1: GPIO 18
# Encoder 2: GPIO 19
gpio_pin = 19

driver = WheelEncoderDriver(gpio_pin)

while 1:
    print(driver._ticks)
    sleep(1)