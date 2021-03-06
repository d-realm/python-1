#!/usr/bin/python3

from time import sleep
from threading import Thread
from multiprocessing import Process, Value
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

# BASIC FUNCTIONS
CLOCKWISE = 1
ANTICLOCKWISE = -1
# To drive
def drive(left, right, time):
    if (time == 0):
        leftMotor.run_direct(duty_cycle_sp = -left)
        rightMotor.run_direct(duty_cycle_sp = -right)
    else:
        leftMotor.run_timed(speed_sp=left, time_sp=time * 1000)
        rightMotor.run_timed(speed_sp=right, time_sp=time * 1000)

def drift():
    drive(55, 100, 1)

# Stops large motors
def brake():
    leftMotor.stop(stop_action='brake')
    rightMotor.stop(stop_action='brake')

def turn(dir, angle):
    """
    Turn in the direction opposite to the contact.
    """

    # We want to turn the robot wheels in opposite directions
    rightMotor.run_timed(speed_sp=dir*-750, time_sp=250*(angle / 46))
    leftMotor.run_timed(speed_sp=dir*750, time_sp=250*(angle / 46))

    # Wait until both motors are stopped:
    while any(m.state for m in (leftMotor, rightMotor)):
        sleep(0.1)




# Connect ultrasonic sensor
# https://sites.google.com/site/ev3python/learn_ev3_python/using-sensors
# https://media.readthedocs.org/pdf/ev3dev-lang/latest/ev3dev-lang.pdf
ultraSensor = UltrasonicSensor()
assert ultraSensor.connected, "Error: Ultrasonic sensor not connected"
ultraSensor.mode = "US-DIST-CM" # This is actually in millimetres

# <HANDLE_TURRET> DONE! (don't touch unless you know what you're doing - Lucas)
# Input constants (they don't need to be global after all!)
SEARCH_DISTANCE = 50 # In centimetres
REVOLUTION = 360
SPEED = REVOLUTION * 1 # In degrees per second

# Output variable
angle_target = Value("i", 0) # The angle (in degrees) that a target was last found at
# 0 = straight ahead, -90 to the left, 90 to the right etc.

# Initialise the turret process, independent from the main program
def init_turret(delay):
    ultraMotor.reset() # Set the current angle to 0
    ultraMotor.stop_action = "brake"
    # Begin a new process which will run concurrently
    p = Process(target=lock_turret, args=(delay, angle_target))
    p.daemon = True
    p.start() # Initialise process

# Turret thread
# Thread runs a huge loop to scan for a target within SEARCH_DISTANCE
def lock_turret(delay, angle):
    ultraMotor = MediumMotor(OUTPUT_C)
    assert ultraMotor.connected, "Error: Ultra motor not connected"
    direction = -1 # Scan direction: 1 = clockwise | -1 = anticlockwise
    distance = 100 # The distance (in centimetres) output of the ultra sensor
    found = False # Whether target has been found
    sleep(delay) # Delay to allow operator to move away before turret rotates
    ultraMotor.run_forever(speed_sp = direction * SPEED) # Scan to the left
    while True: # Huge loop
        # Prevent tangling of ultrasonic sensor cable
        if (ultraMotor.position > REVOLUTION or ultraMotor.position < -REVOLUTION):
            print("turret: fix tangle")
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
        # Then keep rotating and scanning for a target within SEARCH_DISTANCE
        distance = ultraSensor.value() # Output distance in millimetres
        # Found a target! Store in angle
        if (distance <= SEARCH_DISTANCE * 10 and found == False):
            angle.value = fix_angle(ultraMotor.position)
            ultraMotor.stop() # Stop scanning
            found = True
        # Lost the target - scan the immediate area (within 45 degrees)
        elif (distance > SEARCH_DISTANCE * 10 and found == True):
            print("turret: lost target")
            # Keep scanning briefly in the direction it was scanning before
            ultraMotor.position_sp = ultraMotor.position + direction * REVOLUTION /4
            ultraMotor.run_to_abs_pos(speed_sp = SPEED)
            found = False
            while any(ultraMotor.state):
                sleep(0.05)
                if (distance <= SEARCH_DISTANCE * 10): # Found the target again!
                    angle.value = fix_angle(ultraMotor.position)
                    ultraMotor.stop() # Stop scanning
                    found = True
            # The target changed direction, resume scan in the opposite direction
            if (found == False): # Couldn't find target
                sleep(0.2) # Delay after stopping
                direction *= -1 # Switch scan direction and resume scanning
                ultraMotor.run_forever(speed_sp = direction * SPEED)
        sleep(0.02)

# Change {-360 <= angle <= 360} to {-180 <= angle <= 180}
def fix_angle(angle):
    # {-180 <= angle <= 180} 270 degrees clockwise = 90 degrees anticlockwise
    if (angle < -180):
        angle += 360
    elif (angle > 180):
        angle -= 360
    print("turret: found target")
    print("(target at %d degrees)" % angle)
    return angle
# Can now obtain location of target from angle_target.value
# </HANDLE_TURRET>


# Initialise the chase thread, independent from the main program
chase = True
def init_chase(delay):
    t = Thread(target=chase_target, args=(delay,))
    t.setDaemon(True)
    t.start() # Initialise thread

def chase_target(delay):
    global chase
    last = 0 # Angle the target was last chased at
    sleep(delay)
    while True:
        angle = angle_target.value
        # If the robot isn't directly facing the opponent
        # stop and straighten up
        if (chase == True and (angle <= -15 or angle >= 15) and angle != last):
            print("chase: chasing target at %d degrees" % angle)
            rightMotor.stop()
            leftMotor.stop()
            sleep(0.2) # Delay after stopping
            if (angle < 0):
                # Turn robot clockwise
                print("chase: turning clockwise")
                turn(CLOCKWISE, angle * -1)
                # drive(100, 100, 0)
            else:
                # Turn robot anticlockwise
                print("chase: turning anticlockwise")
                turn(ANTICLOCKWISE, angle)
                # drive(100, 100, 0)
            print("chase: finished turning")
            print("chase: (target now at %d degrees)" % angle)
            last = angle
        # If opponent is straight ahead, charge
        if (chase == True and (angle > -15 or angle < 15)):
            print("drive: locked on target, chasing")
            #drive(100,100, 0)
        sleep(0.05)
        print("(chase: target now at %d degrees)" % angle)


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


def check_ring():
    global chase
    colors = ("unknown", "black", "blue", "green", "yellow", "red", "white", "brown")
    if (colorSensor.value() == 1):
        chase = False
        print("black found")
        # Reverse
        print("drive: reversing")
        #drive(-100, -100, 0)
        sleep(1.1)
        # Turn
        # ((ultraSensor.value() / 10) >= SEARCH_DISTANCE):
        #drive(100, 100, 0)
        print("drive: turning")
        #turn(CLOCKWISE, 180)
        #brake()
        print("check_ring complete")
        chase = True

def aesthetics():
    # Makes the robot say something
    Sound.speak('Exterminate').wait()
    sleep(0.5)
    Sound.speak('Exterminate').wait()
    sleep(1)
    Sound.speak('Prepare for doom').wait()
    # Is the sound really necessary?

def aesthetics_thread():
    t = Thread(target=aesthetics)
    t.setDaemon(True)
    t.start()

# aesthetics_thread()
init_turret(0.1) # Begin tracking target after 2 seconds of delay
sleep(3)
# start()
init_chase(0.1)
# Run the robot until a button is pressed.
while not (btn.any()):
    check_ring()

# Stop the motors before exiting.
rightMotor.stop()
leftMotor.stop()
ultraMotor.stop()
