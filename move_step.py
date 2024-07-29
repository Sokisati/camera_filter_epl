from camera_filter_epl import *
import sys

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


servoSpeed *= direction

servo = Servo(servoPWMPin, servoSpeed)
encoderAndDisc = EncoderAndDisc(encoderInputPin, stepCountOnDisc)
system = System(delayBetweenStep, servo, encoderAndDisc, highToLow, initialDrive, plusStep)

system.goFor(stepToTravel)

system.cleanup()
