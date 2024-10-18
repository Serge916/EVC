#!/usr/bin/env python3

from motorDriver import *
from time import sleep

# Motor values
velocity = .1 # Must be contained in [-1,1]

motor = DaguWheelsDriver()
motor.set_wheels_speed(left=velocity, right=velocity)

while 1:
    sleep(10)
