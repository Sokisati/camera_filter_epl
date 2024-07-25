from logging import warn
from gpiozero import PWMOutputDevice
import RPi.GPIO as GPIO
import time
import sys

#change what you need here.
#comment/uncomment in main to debug.
#don't mess around with other stuff! if you really-really need to, call or text me.
servoPWMPin = 13
servoSpeed = 35
        
encoderInputPin = 12
stepCountOnDisc = 8
        
delayBetweenStep = 0.01

highToLow = True
initialDrive = True
plusStep = 0
#plus step is supposed to be 0 if initial drive is true and 1 if initial drive is false
#but I am not entirely sure, so it's there


class Servo:
    def __init__(self, pwmPin, speed,direction):
        self.servo = PWMOutputDevice(pwmPin, frequency=50) 
        self.speed = 90 + (speed*direction)
        
        self.servo.value = 0

    def testMotor(self, forSecond):
        self.servo.value = self._calculateDutyCycle(self.speed)
        time.sleep(forSecond)
        self.servo.value = self._calculateDutyCycle(90)  
        
    def driveMotor(self):
        self.servo.value = self._calculateDutyCycle(self.speed)
    
    def stopMotor(self):
        self.servo.value = 0  

    def _calculateDutyCycle(self, angle):

        return (angle / 180) * (0.1) + 0.02

class EncoderAndDisc:

    def __init__(self, inputPin, stepCountOnDisc):
        self.inputPin = inputPin
        self.stepCountOnDisc = stepCountOnDisc
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.inputPin, GPIO.IN,GPIO.PUD_DOWN)
           
    def setup(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.inputPin, GPIO.IN,GPIO.PUD_DOWN)
    
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
            print(GPIO.input(self.inputPin));

class System:
    def __init__(self, delayBetweenStep, servo, encoderAndDisc, highToLow,initialDrive,plusStep):
        
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
        if initialDrive:
            self.servo.driveMotor()
        
        for i in range(int(stepToTravel)+self.plusStep):  
            print("step: "+str(i))
            if self.highToLow:
                self.driveMotorUntilSignalHL()
            else:
                self.driveMotorUntilSignalLH();
            
            time.sleep(self.delayBetweenStep)
        
        self.servo.stopMotor()
        
    def goFor(self,stepToTravel): 
        
        if initialDrive:
            self.servo.driveMotor()
        
        for i in range(int(stepToTravel)+self.plusStep):  
            print("step: "+str(i))
            if self.highToLow:
                self.driveMotorUntilSignalHL()
            else:
                self.driveMotorUntilSignalLH();
            
            time.sleep(self.delayBetweenStep)
        
        self.servo.stopMotor()
        

    def cleanup(self):
        if hasattr(self, 'client_socket'):
            self.client_socket.close()
        self.socket.close()
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
    exit();

direction = 1

numberOfArgs = len(sys.argv)

if numberOfArgs<2 or sys.argv[1]>25 or (not isInt(sys.argv[1])) or numberOfArgs>4:
    warnAndExit();


stepToTravel = sys.argv[1]

if numberOfArgs==3:
    sys.argv[2]==direction
    
elif numberOfArgs==4:
    sys.argv[2]==direction
    sys.argv[3]==initialDrive
    
    if (sys.argv[3]!=0 or sys.argv[3]!=1):
        warnAndExit();
    else:
        if sys.argv[3]==0:
            initialDrive=False
        else:
            initialDrive=True
    

if (direction!=1 or direction!=-1):
    warnAndExit();


servo = Servo(servoPWMPin,servoSpeed,direction)

encoderAndDisc = EncoderAndDisc(encoderInputPin, stepCountOnDisc)

system = System(delayBetweenStep, servo, encoderAndDisc
                ,highToLow,initialDrive,plusStep)

system.goFor(stepToTravel)

#orderList = ['6','G','4','B'];
#system.filterProcedure(orderList);

#system.encoderAndDisc.printSignal();
