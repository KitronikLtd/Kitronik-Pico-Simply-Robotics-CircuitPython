# AllMotorTest.py
# Test code that ramps each motor 0-100-0 then changes direction and does it again.
# All motors run at once, but with staggered timings

from SimplyRobotics import KitronikSimplyRobotics
from time import sleep

board = KitronikSimplyRobotics()
directions = ["f", "r"]

while True:
    for direction in directions:
        for speed in range(0, 25):
            board.motors[0].on(direction, speed)
            board.motors[1].on(direction, 25 - speed)
            board.motors[2].on(direction, 50 - speed)
            board.motors[3].on(direction, 75 - speed)
            # Ramp speed over 25x100ms => approx 2.5 second.
            sleep(0.1)
            
        for speed in range(0, 25):
            board.motors[0].on(direction, 25 + speed)
            board.motors[1].on(direction, speed)
            board.motors[2].on(direction, 25 - speed)
            board.motors[3].on(direction, 50 - speed)
            sleep(0.1)
            
        for speed in range(0, 25):
            board.motors[0].on(direction, 50 + speed)
            board.motors[1].on(direction, 25 + speed)
            board.motors[2].on(direction, speed)
            board.motors[3].on(direction, 25 - speed)
            sleep(0.1)
            
        for speed in range(0, 25):
            board.motors[0].on(direction, 75 + speed)
            board.motors[1].on(direction, 50 + speed)
            board.motors[2].on(direction, 25 + speed)
            board.motors[3].on(direction, speed)
            sleep(0.1)
            
        for speed in range(0, 25):
            board.motors[0].on(direction, 100 - speed)
            board.motors[1].on(direction, 75 + speed)
            board.motors[2].on(direction, 50 + speed)
            board.motors[3].on(direction, 25 + speed)
            sleep(0.1)
            
        for speed in range(0, 25):
            board.motors[0].on( direction, 75 - speed)
            board.motors[1].on(direction, 100 - speed)
            board.motors[2].on(direction, 75 + speed)
            board.motors[3].on(direction, 50 + speed)
            sleep(0.1)
            
        for speed in range(0, 25):
            board.motors[0].on(direction, 50 - speed)
            board.motors[1].on(direction, 75 - speed)
            board.motors[2].on(direction, 100 - speed)
            board.motors[3].on(direction, 75 + speed)
            sleep(0.1)
            
        for speed in range(0, 25):
            board.motors[0].on(direction, 25 - speed)
            board.motors[1].on(direction, 50 - speed)
            board.motors[2].on(direction, 75 - speed)
            board.motors[3].on(direction, 100 - speed)
            sleep(0.1)
