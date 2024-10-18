from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306
from time import sleep

serial = i2c(port=1, address=0x3C)
device = ssd1306(serial, rotate=2)  # rotate=2 is 180 degrees
with canvas(device) as draw:
    draw.text((10, 5), "Battery: ", fill="white")
    draw.text((10, 15), "Connection: ", fill="white")
    draw.text((10, 25), "ToF: ", fill="white")

while 1:
    sleep(5)
