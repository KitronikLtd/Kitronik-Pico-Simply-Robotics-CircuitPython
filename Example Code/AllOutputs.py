from SimplyRobotics import KitronikSimplyRobotics
from time import sleep

board = KitronikSimplyRobotics()
directions = ["f", "r"]

while True:
    print("motors")
    
    for direction in directions:
        for speed in range(0, 100):
            for motor in range(0, 4):
                board.motors[motor].on(direction, speed)
            
            # Ramp speed over 25x100ms => approx 2.5 second.
            sleep(0.1)
            print(speed)
            
        sleep(0.1)
        
        for speed in range(100, 0, -1):
            for motor in range(0, 4):
                board.motors[motor].on(direction, speed)
            
            # Ramp speed over 25x100ms => approx 2.5 second.
            sleep(0.1)
            print(speed)
        
        sleep(1)
    
    print("servos")
        
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
