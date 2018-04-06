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

ultraMotor.reset() # Set the current angle to 0
ultraMotor.stop_action = "brake"

SEARCH_DISTANCE = 50 # In centimetres
REVOLUTION = 360
SPEED = REVOLUTION * 0.5 # In degrees per second

direction = -1 # Scan direction: 1 = clockwise | -1 = anticlockwise
distance = 100 # The distance (in centimetres) output of the ultra sensor

sleep(3)
ultraMotor.run_forever(speed_sp = direction * SPEED) # Scan to the left

while not (btn.any()):
    # Output ultrasonic sensor value to console
    print("%d" % ultraSensor.value())
    if (ultraSensor.value() < SEARCH_DISTANCE * 10):
        Sound.beep()
    # Prevent tangling of ultrasonic sensor cable
    if (ultraMotor.position > REVOLUTION or ultraMotor.position < -REVOLUTION):
        ultraMotor.stop() # Stop further rotation
        sleep(0.2) # Delay after stopping
        if (ultraMotor.position > 0): # Rotate opposite direction
            direction = -1 # Rotate anticlockwise
        else:
            direction = 1 # Rotate clockwise
        ultraMotor.position_sp = direction * REVOLUTION /8
        ultraMotor.run_to_abs_pos(speed_sp = SPEED)
        while any(ultraMotor.state): # Wait until finished rotating
            sleep(0.05)
        # Continue rotating (and scanning) in direction
        ultraMotor.run_forever(speed_sp = direction * SPEED)
        direction *= -1 # Reset to previous direction
    # Delay between outputs
    sleep(0.02)

ultraMotor.stop()
