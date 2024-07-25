from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
import RPi.GPIO as GPIO
import time
import sys

# Configuration settings
servoPWMPin = 13
servoSpeed = 35
encoderInputPin = 6
stepCountOnDisc = 8
delayBetweenStep = 0.01
highToLow = True
initialDrive = True
plusStep = 0

class Servo:
    def __init__(self, pwmPin, speed):
        self.factory = PiGPIOFactory()
        self.speed = 90 + speed
        self.servo = AngularServo(pwmPin, min_angle=0, max_angle=180, pin_factory=self.factory)
        self.pwmPin = pwmPin

    def testMotor(self, forSecond):
        self.servo.angle = self.speed
        time.sleep(forSecond)
        self.servo.angle = self.stopAngle
        
    def driveMotor(self):
        self.servo.angle = self.speed
    
    def stopMotor(self):
        self.servo.angle = None

class EncoderAndDisc:
    def __init__(self, inputPin, stepCountOnDisc):
        self.inputPin = inputPin
        self.stepCountOnDisc = stepCountOnDisc
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.inputPin, GPIO.IN, GPIO.PUD_DOWN)

    def angleToStep(self, angle):
        return angle / (360 / self.stepCountOnDisc)

    def waitForHighToLow(self):
        while GPIO.input(self.inputPin) == 0:
            pass
        while GPIO.input(self.inputPin) == 1:
            pass

    def waitForLowToHigh(self):
        while GPIO.input(self.inputPin) == 1:
            pass
        while GPIO.input(self.inputPin) == 0:
            pass

    def printSignal(self):
        while True:
            print(GPIO.input(self.inputPin))

class System:
    def __init__(self, delayBetweenStep, servo, encoderAndDisc, highToLow, initialDrive, plusStep):
        self.delayBetweenStep = delayBetweenStep
        self.servo = servo
        self.encoderAndDisc = encoderAndDisc
        self.highToLow = highToLow
        self.initialDrive = initialDrive
        self.plusStep = plusStep

    def driveMotorUntilSignalHL(self):
        self.servo.driveMotor()
        self.encoderAndDisc.waitForHighToLow()
        self.servo.stopMotor()

    def driveMotorUntilSignalLH(self):
        self.servo.driveMotor()
        self.encoderAndDisc.waitForLowToHigh()
        self.servo.stopMotor()

    def goToAngle(self, angle):
        stepToTravel = self.encoderAndDisc.angleToStep(angle)
        if self.initialDrive:
            self.servo.driveMotor()

        for i in range(int(stepToTravel) + self.plusStep):
            print(f"step: {i+1}")
            if self.highToLow:
                self.driveMotorUntilSignalHL()
            else:
                self.driveMotorUntilSignalLH()

            time.sleep(self.delayBetweenStep)

        self.servo.stopMotor()

    def goFor(self, stepToTravel):
        if self.initialDrive:
            self.servo.driveMotor()

        for i in range(int(stepToTravel) + self.plusStep):
            print(f"step: {i}")
            if self.highToLow:
                self.driveMotorUntilSignalHL()
            else:
                self.driveMotorUntilSignalLH()

            time.sleep(self.delayBetweenStep)

        self.servo.stopMotor()

    def cleanup(self):
        GPIO.cleanup()
        print("GPIO cleanup completed")
        exit()

def isInt(x):
    try:
        int(x)
        return True
    except ValueError:
        return False

def warnAndExit():
    print("Stop trying to break the program.")
    exit()

if len(sys.argv) < 2 or not isInt(sys.argv[1]) or len(sys.argv) > 4:
    warnAndExit()

stepToTravel = int(sys.argv[1])

direction = 1
if len(sys.argv) >= 3:
    direction = int(sys.argv[2])
    if direction != 1 and direction != -1:
        warnAndExit()

if len(sys.argv) == 4:
    initialDrive = int(sys.argv[3])
    if initialDrive != 0 and initialDrive != 1:
        warnAndExit()
    initialDrive = bool(initialDrive)

servo = Servo(servoPWMPin, servoSpeed, direction)
encoderAndDisc = EncoderAndDisc(encoderInputPin, stepCountOnDisc)
system = System(delayBetweenStep, servo, encoderAndDisc, highToLow, initialDrive, plusStep)

system.goFor(stepToTravel)
