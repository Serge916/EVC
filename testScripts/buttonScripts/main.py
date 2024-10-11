#!/usr/bin/env python3

import time
from buttonClass import ButtonEvent, ButtonDriver

LED_GPIO = 37
SIGNAL_GPIO = 40


def event_cb(event: ButtonEvent):
    if event == ButtonEvent.PRESS:
        print("Press event")
        return
    if event == ButtonEvent.RELEASE:
        print("Release event")
        return


button = ButtonDriver(LED_GPIO, SIGNAL_GPIO, event_cb)
button.led.on()

while 1:
    time.sleep(0.1)
