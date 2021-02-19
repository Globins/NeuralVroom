import time
import board
import digitalio



def rotateMotor(rotationStep, rotationCount, rotationDirection, degrees, IN1, IN2, IN3, IN4):
    while rotationCount != 0:
        if rotationStep == 0:
            IN1.value = False
            IN2.value = False
            IN3.value = False
            IN4.value = True
        elif rotationStep == 1:
            IN1.value = False
            IN2.value = False
            IN3.value = True
            IN4.value = True
        elif rotationStep == 2:
            IN1.value = False
            IN2.value = False
            IN3.value = True
            IN4.value = False
        elif rotationStep == 3:
            IN1.value = False
            IN2.value = True
            IN3.value = True
            IN4.value = False
        elif rotationStep == 4:
            IN1.value = False
            IN2.value = True
            IN3.value = False
            IN4.value = False
        elif rotationStep == 5:
            IN1.value = True
            IN2.value = True
            IN3.value = False
            IN4.value = False
        elif rotationStep == 6:
            IN1.value = True
            IN2.value = False
            IN3.value = False
            IN4.value = False    
        elif rotationStep == 7:
            IN1.value = True
            IN2.value = False
            IN3.value = False
            IN4.value = True
        else:
            IN1.value = False
            IN2.value = False
            IN3.value = False
            IN4.value = False

        rotationCount -= 1 
        rotationStep += rotationDirection
        degrees += (.9*rotationDirection)

        if rotationStep > 7:
            rotationStep = 0
        elif rotationStep < 0:
            rotationStep = 7
        
        if degrees > 359.2:
            degrees = 0
        elif degrees < .5:
            degrees = 360
        
        time.sleep(0.0008)
    IN1.value = False
    IN2.value = False
    IN3.value = False
    IN4.value = False

    return [rotationStep, degrees]

