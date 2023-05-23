'''
CircuitPython Library for the Kitronik Simply Robotics board for Pico.
www.kitronik.co.uk/5348

API:
    servos[] - array of 8 servos
        servos[WHICH_SERVO].goToPosition(degrees): Sets a servo's position in degrees.
        servos[WHICH_SERVO].goToPeriod(period): Sets a servo's position using the pulse length period.
            where:
            WHICH_SERVO - the servo to control (0 - 7)
            degrees - angle to go to (0 - 180)
            period - pulse length to output in uSec (500 - 2500)    
        
    motors[] - array of 4 motors
        motors[WHICH_MOTOR].on(direction, speed): Turns the motor on at a speed in the direction.
        motors[WHICH_MOTOR].off(): Turns the motor off.
            where:
            WHICH_MOTOR - the motor to control (0 - 3)
            direction - either forwards or reverse ("f" or "r")
            speed - how fast to turn the motor (0 - 100)
            
    steppers[] - array of 2 stepper motors
        steppers[WHICH_STEPPER].step(direction): Turns the stepper motor a full step in the direction.
        steppers[WHICH_STEPPER].halfStep(direction): Turns the stepper motor a half step in the direction.
            where:
            WHICH_STEPPER - the stepper motor to control (0 or 1)
            direction - either forwards or reverse ("f" or "r")
        
        Note: stepper 0 should be connected to motors 0 and 1,
              stepper 1 should be connected to motors 3 and 4
'''

from array import array
from board import GP2, GP3, GP4, GP5, GP6, GP7, GP8, GP9, GP12, GP13, GP14, GP15, GP16, GP17, GP18, GP19
from pwmio import PWMOut
from rp2pio import StateMachine
from adafruit_pioasm import Program

p = Program("""
    pull            ;; don't start until first value available
    
.wrap_target
    pull noblock
    mov x, osr      ;; reload OSR with last value
    set pins, 0
    out y, 16       ;; off time
    
loop_off:
    jmp y--, loop_off
    set pins, 1
    out y, 16       ;; on time
    
loop_on:
    jmp y--, loop_on
.wrap
""")

'''
a class which can encapsulate a stepper motor state machine
It makes no assumptions about steps per rev - that is upto the higher level code to do

This class will drive 4 wire, bipolar steppers. 
These have 2 coils which are alternately energised to make a step.
The class is passed the pairs of motors from the board as these are analogous to the coils.
'''
class StepperMotor:
    stepSequence = [["f","-"],
                    ["-","r"],
                    ["r","-"],
                    ["-","f"]]
    halfStepSequence = [["f","-"],
                        ["f","r"],
                        ["-","r"],
                        ["r","r"],
                        ["r","-"],
                        ["r","f"],
                        ["-","f"],
                        ["f","f"]]

    def __init__(self, coilA, coilB):
        self.coils = [coilA, coilB]
        self.state = 0

    # Full stepping is 4 states, each coil only energised in turn and one at once. 
    def step(self, direction = "f"):
        if direction == "f":
            self.state += 1
            
        elif direction == "r":
            self.state -= 1
            
        else:
            # Harsh, but at least you'll know
            raise Exception("INVALID DIRECTION")
            
        if self.state > 3:
            self.state = 0
            
        if self.state < 0:
            self.state = 3
            
        for i in range(2):
            self.coils[i].on(self.stepSequence[self.state][i], 100)
    
    # Half stepping is each coil energised in turn, but sometimes both at ones (holds halfway between positions)
    def halfStep(self, direction = "f"):
        if direction == "f":
            self.state += 1
            
        elif direction == "r":
            self.state -= 1
            
        else:
            # Harsh, but at least you'll know
            raise Exception("INVALID DIRECTION")
            
        if self.state > 7:
            self.state = 0
            
        if self.state < 0:
            self.state = 7
            
        for i in range(2):
            self.coils[i].on(self.halfStepSequence[self.state][i], 100)

# This class provides a simple wrapper to the micropython PWM pins to hold them in a set for each motor
class SimplePWMMotor:
    def __init__(self, forwardPin, reversePin, startfreq = 20):
        self.forwardPin = PWMOut(forwardPin, frequency=startfreq)
        self.reversePin = PWMOut(reversePin, frequency=startfreq)
        self.off()
    
    # Directions are "f" - forwards, "r" - reverse and "-" - off. The inclusion of off makes stepper code simpler
    def on(self, direction, speed = 0):
        # Cap speed to 0-100%
        if speed < 0:
            speed = 0
            
        elif speed > 100:
            speed = 100
  
        # Convert 0-100 to 0-65535
        pwmVal = int(speed * 655.35)
        
        if direction == "f":
            self.forwardPin.duty_cycle = pwmVal
            self.reversePin.duty_cycle = 0
            
        elif direction == "r":
            self.forwardPin.duty_cycle = 0
            self.reversePin.duty_cycle = pwmVal
            
        elif direction == "-":
            self.forwardPin.duty_cycle = 0
            self.reversePin.duty_cycle = 0
            
        else:
            # Harsh, but at least you'll know
            raise Exception("INVALID DIRECTION")
       
    def off(self):
        self.on("-", 0)

'''
Class that controls Serovs using the RP2040 PIO to generate the pulses.

ServoControl:
Servo 0 degrees -> pulse of 0.5ms, 180 degrees 2.5ms
pulse train freq 50hz - 20mS
1uS is freq of 1000000
servo pulses range from 500 to 2500usec and overall pulse train is 20000usec repeat.
'''
class PIOServo:
    degreesToUS = 2000 / 180
 
    def __init__(self, servoPin):
        self.sm = StateMachine(p.assembled, frequency=1_000_000, first_set_pin=servoPin, **p.pio_kwargs)
    
    def goToPosition(self, degrees):
        period = int(degrees * self.degreesToUS + 500)
        self.goToPeriod(period)
    
    def goToPeriod(self, period):
        if period < 500:
            period = 500
        if period > 2500:
            period = 2500
        
        self.sm.background_write(memoryview(array('HH', [20_000 - period, period])).cast('L'))

'''
A class to provide the functionality of the Kitronik 5348 Simply Robotics board.
www.kitronik.co.uk/5348

The motors are connected as
    Motor 1 GP2 + GP5 -
    Motor 2 GP4 + GP3 -
    Motor 3 GP6 + GP9 -
    Motor 4 GP8 + GP7 -
The servo pins are 15,14,13,12,19,18,17,16 for servo 0 -> servo 7
The numbers look strange but it makes the tracking on the PCB simpler and is hidden inside this lib
'''
class KitronikSimplyRobotics:
    def __init__(self, enableStepper0=False, enableStepper1=False):
        stepper0Freq = 20
        if enableStepper0:
            stepper0Freq = 100
        stepper1Freq = 20
        if enableStepper1:
            stepper1Freq = 100
        self.motors = [SimplePWMMotor(GP2, GP5, stepper0Freq),
                       SimplePWMMotor(GP4, GP3, stepper0Freq),
                       SimplePWMMotor(GP6, GP9, stepper1Freq),
                       SimplePWMMotor(GP8, GP7, stepper1Freq)]
        self.steppers = [StepperMotor(self.motors[0], self.motors[1]),
                         StepperMotor(self.motors[2], self.motors[3])]
        self.servos = [PIOServo(GP15),
                       PIOServo(GP14),
                       PIOServo(GP13),
                       PIOServo(GP12),
                       PIOServo(GP19),
                       PIOServo(GP18),
                       PIOServo(GP17),
                       PIOServo(GP16)]
