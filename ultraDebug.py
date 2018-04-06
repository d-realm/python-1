#!/usr/bin/python3

from time import sleep
from ev3dev.ev3 import *

# Will need to check EV3 button state
btn = Button()

# Connect motor
ultraMotor = MediumMotor(OUTPUT_C)
assert ultraMotor.connected, "Error: Ultra motor not connected"

# Connect ultrasonic sensor
ultraSensor = UltrasonicSensor()
assert ultraSensor.connected, "Error: Ultrasonic sensor not connected"
ultraSensor.mode = "US-DIST-CM" # This is actually in millimetres

SEARCH_DISTANCE = 50 # In centimetres
REVOLUTION = 360
SPEED = REVOLUTION * 0.5 # In degrees per second

sleep(3)
ultraMotor.run_forever(speed_sp = -1 * SPEED) # Scan to the left

while not (btn.any()):
    print("%d" % ultraSensor.value())
    if (ultraSensor.value() < SEARCH_DISTANCE * 10):
        Sound.beep()
    sleep(0.02)

ultraMotor.stop()
