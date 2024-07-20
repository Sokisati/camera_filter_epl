from gpiozero import PWMOutputDevice
import RPi.GPIO as GPIO
import time
import socket

class Servo:
    def __init__(self, pwmPin, speed):

        self.servo = PWMOutputDevice(pwmPin, frequency=50) 
        self.speed = 90 + speed
        
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
        GPIO.setup(inputPin, GPIO.IN,GPIO.PUD_DOWN)
    
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
    def __init__(self, delayBetweenStep, servo, encoderAndDisc, port):
        self.delayBetweenStep = delayBetweenStep
        self.servo = servo
        self.encoderAndDisc = encoderAndDisc
        self.colorList = ['N', 'B', 'G', 'R']
        self.filterIndex = 0
        
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
                return step * (360 / listSize)
                self.filterIndex = i
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
        self.servo.driveMotor()
        
        for _ in range(int(stepToTravel)):  
            self.driveMotorUntilSignalHL()
            time.sleep(self.delayBetweenStep)
        
        
        self.servo.stopMotor()
        
    def filterProcedure(self, orderList):
        
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

    def cleanup(self):
        self.client_socket.close()
        self.socket.close()
        GPIO.cleanup()
        print("GPIO cleanup completed")
        exit()

    def mainLoop(self):
        self.socket.bind(('', self.stPort))
        self.socket.listen(1)
        self.client_socket, self.client_address = self.socket.accept()
        print("Connected to satellite")
    
        while True:
            try:
                data = self.client_socket.recv(1024).decode()
                if data != '0':
                    print("Command received")
                    orderList = list(data)
                    self.filterProcedure(orderList)
                    self.cleanup()
            except KeyboardInterrupt:
                print("Program interrupted")
            finally:
                self.cleanup()
                  
servo = Servo(pwmPin=13, speed=30)
encoderAndDisc = EncoderAndDisc(inputPin=12, stepCountOnDisc=8)
system = System(delayBetweenStep=0.01, servo=servo, encoderAndDisc=encoderAndDisc, port=12347)

system.mainLoop();
