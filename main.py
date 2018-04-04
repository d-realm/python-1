#!/usr/bin/python3

from time import sleep
from threading import Thread
import sys, os

from ev3dev.ev3 import *

# Will need to check EV3 button state
btn = Button()

# Connect motors
rightMotor = LargeMotor(OUTPUT_A)
assert rightMotor.connected, "Error: Right motor not connected"
leftMotor  = LargeMotor(OUTPUT_D)
assert leftMotor.connected, "Error: Left motor not connected"
ultraMotor = MediumMotor(OUTPUT_C)
assert ultraMotor.connected, "Error: Ultra motor not connected"

# Connect colour sensor
colorSensor = ColorSensor()
assert colorSensor.connected, "Error: Color sensor not connected"
colorSensor.mode = "COL-COLOR"

# Connect ultrasonic sensor
# https://sites.google.com/site/ev3python/learn_ev3_python/using-sensors
# https://media.readthedocs.org/pdf/ev3dev-lang/latest/ev3dev-lang.pdf
ultraSensor = UltrasonicSensor()
assert ultraSensor.connected, "Error: Ultrasonic sensor not connected"
ultraSensor.mode = "US-DIST-CM" # This is actually in millimetres

# <HANDLE_TURRET> DONE! (don't touch unless you know what you're doing - Lucas)
# Input constants (they don't need to be global after all!)
SEARCH_DISTANCE = 50 # In centimetres
SPEED = 360 # In degrees per second
# Output variable
angle_target = 0 # The angle (in degrees) that a target was last found at
# 0 = straight ahead, -90 to the left, 90 to the right etc.

# Initialise the turret thread, independent from the main program
def init_turret(delay):
    ultraMotor.reset() # Set the current angle to 0
    ultraMotor.stop_action = "brake"
    # https://sites.google.com/site/ev3python/learn_ev3_python/threads
    t = Thread(target=lock_turret, args=(delay,))
    t.setDaemon(True)
    t.start() # Initialise thread

# Turret thread
# Thread runs a huge loop to scan for a target within SEARCH_DISTANCE
def lock_turret(delay):
    direction = -1 # Scan direction: 1 = clockwise | -1 = anticlockwise
    distance = 100 # The distance (in centimetres) output of the ultra sensor
    found = False # Whether target has been found
    sleep(delay) # Delay to allow operator to move away before turret rotates
    ultraMotor.run_forever(speed_sp = direction * SPEED) # Scan to the left
    while True: # Huge loop
        # Prevent tangling of ultrasonic sensor cable
        if (ultraMotor.position > 360 or ultraMotor.position < -360):
            print("turret: fix tangle")
            ultraMotor.stop() # Stop further rotation
            sleep(0.2) # Delay after stopping
            if (ultraMotor.position > 0): # Rotate opposite direction
                direction = -1 # Rotate anticlockwise
            else:
                direction = 1 # Rotate clockwise
            ultraMotor.position_sp = direction * 180 # Rotate 180 degrees
            ultraMotor.run_to_abs_pos(speed_sp = SPEED)
            while any(ultraMotor.state): # Wait until finished rotating
                sleep(0.05)
            # Continue rotating (and scanning) in direction
            ultraMotor.run_forever(speed_sp = direction * SPEED)
            direction *= -1 # Reset to previous direction
        # Then keep rotating and scanning for a target within SEARCH_DISTANCE
        distance = ultraSensor.value() / 10 # Convert to centimetres
        # Found a target! Store in angle_target
        if (distance <= SEARCH_DISTANCE and found == False):
            found_target()
            found = True
        # Lost the target - scan the immediate area (within 45 degrees)
        elif (distance > SEARCH_DISTANCE and found == True):
            print("turret: lost target")
            # Keep scanning briefly in the direction it was scanning before
            ultraMotor.position_sp = ultraMotor.position + direction * 45
            ultraMotor.run_to_abs_pos(speed_sp = SPEED)
            found = False
            while any(ultraMotor.state):
                sleep(0.05)
                if (distance <= SEARCH_DISTANCE): # Found the target again!
                    found_target()
                    found = True
            # The target changed direction, resume scan in the opposite direction
            if (found == False): # Couldn't find target
                sleep(0.2) # Delay after stopping
                direction *= -1 # Switch scan direction and resume scanning
                ultraMotor.run_forever(speed_sp = direction * SPEED)
        sleep(0.02)

# Found start of target, store it into angle_target
def found_target(): # (replaced the averaging algorithm with a delay)
    global angle_target
    # sleep(0.1) # Continue rotating briefly, estimating centre of target
    angle_target = ultraMotor.position
    ultraMotor.stop()
    print("turret: found target")
# Can now obtain location of target from angle_target
# </HANDLE_TURRET>

#def chase_target():
    #global angle_target
    #if (angle_target < 0):
        # Turn robot clockwise
    #else:
        # Turn robot anticlockwise

def start():
    print("Enter right motor speed then left motor speed.")
    x = 0
    y = 0
    x = input()
    y = input()
    x = int(x)
    y = int(y)
    rightMotor.run_direct(duty_cycle_sp = -x)
    leftMotor.run_direct(duty_cycle_sp = -y)

def turn(dir):
    """
    Turn in the direction opposite to the contact.
    """

    # We want to turn the robot wheels in opposite directions
    rightMotor.run_timed(speed_sp=dir*-750, time_sp=250)
    leftMotor.run_timed(speed_sp=dir*750, time_sp=250)

    # Wait until both motors are stopped:
    while any(m.state for m in (leftMotor, rightMotor)):
        sleep(0.1)

def check_ring():
    colors = ("unknown", "black", "blue", "green", "yellow", "red", "white", "brown")
    print(colors[colorSensor.value()])
    if (colorSensor.value() == 1):
        print("black")
        rightMotor.run_direct(duty_cycle_sp = 0)
        leftMotor.run_direct(duty_cycle_sp = 0)
        sleep(0.1)
        rightMotor.run_direct(duty_cycle_sp = -75)
        leftMotor.run_direct(duty_cycle_sp = -75)
        turn(4)
        rightMotor.run_direct(duty_cycle_sp = 0)
        leftMotor.run_direct(duty_cycle_sp = 0)

# Makes the robot say something
#Sound.speak('Exterminate').wait()
#sleep(0.5)
#Sound.speak('Exterminate').wait()
#sleep(1)
#Sound.speak('Prepare for doom').wait()
Sound.tone([(784, 200, 100),
            (784, 200, 100),
            (784, 200, 100),
            (659, 2000, 1300),
            (698, 200, 100),
            (698, 200, 100),
            (698, 200, 100),
            (587, 3000, 2000),
            (262, 200, 100),
            (784, 200, 100),
            (784, 200, 100),
            (784, 200, 100),
            (659, 200, 100),
            (880, 200, 100),
            (880, 200, 100),
            (880, 200, 100),
            (784, 200, 100),
            (1319, 200, 100),
            (1319, 200, 100),
            (1319, 200, 100),
            (1047, 800, 400)])
init_turret(2) # Begin tracking target after 2 seconds of delay
sleep(3)
start()
# Run the robot until a button is pressed.
while not (btn.any()):
    check_ring()
    #chase_target()

# Stop the motors before exiting.
rightMotor.stop()
leftMotor.stop()
ultraMotor.stop()
