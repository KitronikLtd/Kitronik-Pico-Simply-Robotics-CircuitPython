# AllServoTest.py
# Test code that ramps each servo from 0-180-0

from SimplyRobotics import KitronikSimplyRobotics
from time import sleep

board = KitronikSimplyRobotics()

while True:
    for degrees in range(180):
        for servo in range(8):
            board.servos[servo].goToPosition(degrees)
        
        # Ramp speed over 10x180ms => approx 2 seconds.
        sleep(0.01)
        
    for degrees in range(180):
        for servo in range(8):
            board.servos[servo].goToPosition(180 - degrees)
        
        # Ramp speed over 10x180ms => approx 2 seconds.
        sleep(0.01)
