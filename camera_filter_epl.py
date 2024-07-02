from gpiozero import AngularServo
import RPi.GPIO as GPIO
import time
import socket
import json



class Servo:
    def __init__(self, pwmPin, speed):
        self.servo =AngularServo(18, min_angle=0, max_angle=180, min_pulse_width=0.0005, max_pulse_width=0.0025)
        
        self.pwmPin = pwmPin
        self.speed = 90 + speed
   
    def testMotor(self, forSecond):
        self.servo.angle(self.speed)
        time.sleep(forSecond)
        self.servo.angle(self.speed)
        
    def driveMotor(self):
        self.servo.angle(self.speed)
    
    def stopMotor(self):
        self.servo.angle(90)
       
class EncoderAndDisc:
    def __init__(self, inputPin, stepCountOnDisc):
        self.inputPin = inputPin
        self.stepCountOnDisc = stepCountOnDisc
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.inputPin, GPIO.IN)
    
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
   
class System:
    def __init__(self, delayBetweenStep, servo, encoderAndDisc,port):
        self.delayBetweenStep = delayBetweenStep
        
        self.servo = servo
        self.encoderAndDisc = encoderAndDisc
        
        self.colorList = ['N', 'B', 'G', 'R']
        self.filterIndex = 0
        
        self.stIp = '127.0.0.1'
        self.stPort = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        self.socket.bind('',self.stPort); 
    
        self.client_socket, self.client_address = socket.accept();
    
        print("Camera filter is ready and listening for incoming orders");
        
    
    def returnAngleForColor(self, color):
        step = 0
        listSize = len(self.colorList)
        i = self.filterIndex

        while True:
            if self.colorList[i] == color:
                return step * (360 / listSize);
            step += 1;
            i = (i + 1) % listSize; 
            if step > listSize: 
                break;
        return None           

    def driveMotorUntilSignalHL(self):
        self.servo.driveMotor()
        self.encoderAndDisc.waitForHighToLow()
        self.servo.stopMotor()
        
    def goToAngle(self, angle):
        
        stepToTravel = self.encoderAndDisc.angleToStep(angle)
        self.servo.driveMotor()
        
        for _ in range(stepToTravel):
            self.driveMotorUntilSignalHL()
            time.sleep(self.delayBetweenStep)
        
        #just to be sure
        self.servo.stopMotor()
        
    def filterProcedure(self,orderList):
        
        
        #first color
        angleToTravel = self.returnAngleForColor(orderList[1]);
        self.goToAngle(angleToTravel);
        time.sleep(orderList[0])

        #second color
        angleToTravel = self.returnAngleForColor(orderList[3]);
        self.goToAngle(angleToTravel);
        time.sleep(orderList[2])

        #back to neutral
        angleToTravel = self.returnAngleForColor(orderList[3]);
        self.goToAngle(angleToTravel);

    def mainLoop(self):
        while True:
            data = self.client_socket.recv(1024).decode();
            if(len(data)==4):
                self.filterProcedure(data);
                exit();
       

     
servo = Servo(5, 30)
encoderAndDisc = EncoderAndDisc(5, 8)
system = System(0.01, 0.5, servo, encoderAndDisc)

print(system.returnAngleForColor('N'))
