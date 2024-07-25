from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory

import RPi.GPIO as GPIO
import time
import socket
import json

#change what you need here.
#comment/uncomment in main to debug.
#don't mess around with other stuff! if you really-really need to, call or text me.
servoPWMPin = 13
servoSpeed = 25
        
encoderInputPin = 6
stepCountOnDisc = 8
        
delayBetweenStep = 0.01
portToListen = 12347

highToLow = True
initialDrive = True
plusStep = 0
#plus step is supposed to be 0 if initial drive is true and 1 if initial drive is false
#but I am not entirely sure, so it's there


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
    def __init__(self, delayBetweenStep, servo, encoderAndDisc, port, highToLow,initialDrive,plusStep):
        
        self.delayBetweenStep = delayBetweenStep
        self.servo = servo
        self.encoderAndDisc = encoderAndDisc
        self.colorList = ['N', 'B', 'G', 'R']
        self.filterIndex = 0
        self.highToLow = highToLow
        self.initialDrive = initialDrive
        self.plusStep = plusStep
        
        self.stIp = '127.0.0.1'
        self.stPort = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        print("Camera filter is ready and listening for incoming orders")
    
    def returnAngleForColor(self, color):
        step = 0
        listSize = len(self.colorList)
        i = self.filterIndex

        while True:
            if self.colorList[i] == color:
                self.filterIndex = i
                return step * (360 / listSize)
            step += 1
            i = (i + 1) % listSize
            if step > listSize:
                break
            
        return None           

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
        
        for i in range(int(stepToTravel)+self.plusStep):  
            print("step: "+str(i+1))
            if self.highToLow:
                self.driveMotorUntilSignalHL()
            else:
                self.driveMotorUntilSignalLH();
            
            time.sleep(self.delayBetweenStep)
        
        self.servo.stopMotor()
        
    def filterProcedure(self, orderList):

        #I have no idea why but connecting to main flight program causes button pin to broke
        self.encoderAndDisc.setup();        

        # first color
        angleToTravel = self.returnAngleForColor(orderList[1])  
        print(angleToTravel);
        self.goToAngle(angleToTravel)
        time.sleep(int(orderList[0]))

        # second color
        angleToTravel = self.returnAngleForColor(orderList[3])
        print(angleToTravel);
        self.goToAngle(angleToTravel)
        time.sleep(int(orderList[2]))

        # back to neutral
        angleToTravel = self.returnAngleForColor('N')  
        print(angleToTravel);
        self.goToAngle(angleToTravel)
        self.cleanup();

    def cleanup(self):
        if hasattr(self, 'client_socket'):
            self.client_socket.close()
        self.socket.close()
        GPIO.cleanup()
        print("GPIO cleanup completed")
        exit()

    def mainLoop(self):
        
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(('', self.stPort))
        self.socket.listen(1)
        self.client_socket, self.client_address = self.socket.accept()
        print("Connected to satellite")

        try:
            while True:
                data = self.client_socket.recv(1024).decode()
                if data != 0 :
                    print("Command received")
                    orderList = list(json.loads(data))
                    print(orderList)
                    self.filterProcedure(orderList)
        except KeyboardInterrupt:
            print("Interrupted by user")
        finally:
            self.cleanup()

                  
servo = Servo(servoPWMPin, servoSpeed)

encoderAndDisc = EncoderAndDisc(encoderInputPin, stepCountOnDisc)

system = System(delayBetweenStep, servo, encoderAndDisc,
                portToListen,highToLow,initialDrive,plusStep)

#system.mainLoop();
system.servo.driveMotor();
time.sleep(2);
system.servo.stopMotor();
time.sleep(2);

system.servo.driveMotor();
time.sleep(2);
system.servo.stopMotor();
time.sleep(2);

#orderList = ['6','G','4','B'];
#system.filterProcedure(orderList);

#system.encoderAndDisc.printSignal();
