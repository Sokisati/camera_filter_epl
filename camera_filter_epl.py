from module_file import *

servo = Servo(servoPWMPin, servoSpeed)

encoderAndDisc = EncoderAndDisc(encoderInputPin, stepCountOnDisc,delayForSignal)

system = System(delayBetweenStep, servo, encoderAndDisc,
                portToListen,highToLow,initialDrive,plusStep,revertLastSignal)

#system.mainLoop();
system.servo.stopMotor()
#orderList = ['6','G','4','B'];
#system.filterProcedure(orderList);

#system.encoderAndDisc.printSignal();
