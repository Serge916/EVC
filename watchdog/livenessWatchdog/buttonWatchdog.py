#!/usr/bin/env python2

import time
import serial
import subprocess
from buttonClass import ButtonEvent, ButtonDriver

LED_GPIO = 37
SIGNAL_GPIO = 40
SHUTDOWN_COMMAND = b"QQ"


def shutdown_duckiebattery(serial_port="/dev/ttyACM0", baud_rate=9600):
    """
    Sends a shutdown command to the Duckiebattery via USB serial connection.
    """
    # Command to be sent
    command = SHUTDOWN_COMMAND
    try:
        # Open serial connection to the Duckiebattery
        ser = serial.Serial(port=serial_port, baudrate=baud_rate, timeout=1)
        print(
            "Connected to Duckiebattery on {} at {} baud.".format(
                serial_port, baud_rate
            )
        )

        # Send the shutdown command
        ser.write(command)
        print("Shutdown command sent to Duckiebattery.")

        # Optional: Read response from Duckiebattery
        response = ser.readline().decode("utf-8").strip()
        if response:
            print("Duckiebattery response: {}".format(response))

        # Close the serial connection
        ser.close()

    except serial.SerialException as e:
        print("Serial exception occurred: {}".format(e))
    except Exception as e:
        print("An error occurred while sending the shutdown command: {}".format(e))


class button:
    def __init__(self):
        self.driver = ButtonDriver(LED_GPIO, SIGNAL_GPIO, self.event_cb)
        self.ledState = 1
        self.driver.led.set(self.ledState)
        self.commandedShutdown = False
        self.pressTS = time.time()

    def event_cb(self, event):
        if event == ButtonEvent.PRESS:
            print("Press event")
            self.pressTS = time.time()
            return
        if event == ButtonEvent.RELEASE:
            print("Release event")
            self.ledState ^= 1
            self.driver.led.set(self.ledState)
            releaseTS = time.time()
            print(
                "first event: {}, second event: {} => {}".format(
                    self.pressTS, releaseTS, self.pressTS - releaseTS
                )
            )
            if (releaseTS - self.pressTS) > 3:
                print("Long press")
                self.commandedShutdown = True
            return


try:
    button = button()

    while not button.commandedShutdown:
        time.sleep(2)
finally:
    button.driver.led.set(0)
    shutdown_duckiebattery()
    subprocess.call(["sudo", "shutdown", "now"])
