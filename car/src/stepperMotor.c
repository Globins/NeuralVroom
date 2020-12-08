#include "stepperMotor.h"
#include <stdio.h>
#include <stdlib.h>
#include <pigpio.h>




struct StepperMotor* setUpStepperMotor(unsigned A, unsigned A1, unsigned B, unsigned B1){
    struct StepperMotor* motor = malloc(sizeof(struct StepperMotor));
    if(gpioSetMode(A, PI_OUTPUT) || gpioSetMode(A1, PI_OUTPUT) || gpioSetMode(B, PI_OUTPUT) || gpioSetMode(B1, PI_OUTPUT)){
        perror("failed to set stepperMotor pins");
        exit(1);
    }
    motor->A = A;
    motor->A1 = A1;
    motor->B = B;
    motor->B1 = B1;
    motor->rotationStep = 0;
    motor->degrees = 0;
    return motor;
}

void cleanUpMotor(struct StepperMotor* motor){
    free(motor);
}

void rotateMotor(struct StepperMotor* motor, int rotationCount, int rotationDirection){
    while(rotationCount){
        switch (motor->rotationStep)
        {
            case 0:
                gpioWrite(motor->A, HIGH);
                gpioWrite(motor->A1, LOW);
                gpioWrite(motor->B, LOW);
                gpioWrite(motor->B1, LOW);
                break;
            case 1:
                gpioWrite(motor->A, HIGH);
                gpioWrite(motor->A1, LOW);
                gpioWrite(motor->B, HIGH);
                gpioWrite(motor->B1, LOW);
                break;
            case 2:
                gpioWrite(motor->A, LOW);
                gpioWrite(motor->A1, LOW);
                gpioWrite(motor->B, HIGH);
                gpioWrite(motor->B1, LOW);
                break;
            case 3:
                gpioWrite(motor->A, LOW);
                gpioWrite(motor->A1, HIGH);
                gpioWrite(motor->B, HIGH);
                gpioWrite(motor->B1, LOW);
                break;
            case 4:
                gpioWrite(motor->A, LOW);
                gpioWrite(motor->A1, HIGH);
                gpioWrite(motor->B, LOW);
                gpioWrite(motor->B1, LOW);
                break;
            case 5:
                gpioWrite(motor->A, LOW);
                gpioWrite(motor->A1, HIGH);
                gpioWrite(motor->B, LOW);
                gpioWrite(motor->B1, HIGH);
                break;
            case 6:
                gpioWrite(motor->A, LOW);
                gpioWrite(motor->A1, LOW);
                gpioWrite(motor->B, LOW);
                gpioWrite(motor->B1, HIGH);
                break;    
            default:
                gpioWrite(motor->A, HIGH);
                gpioWrite(motor->A1, LOW);
                gpioWrite(motor->B, LOW);
                gpioWrite(motor->B1, HIGH);
                break;
        }
        rotationCount--;
        motor->rotationStep += rotationDirection;
        motor->degrees += (.9*rotationDirection);
        if(motor->rotationStep > 7){
            motor->rotationStep = 0;
        }else if(motor->rotationStep < 0){
            motor->rotationStep = 7;
        }
        if(motor->degrees > 359.2){
            motor->degrees = 0;
        }else if(motor->degrees < .5){
            motor->rotationStep = 7;
        }
        gpioSleep(PI_TIME_RELATIVE, 0, 5000);
    }
}

void calibrateMotor(struct StepperMotor* motor){
    int quit;
    int steps;
    printf("enter calibrations values((+/-)steps, endCalibrationFlag))\n");
    while(1)
    {
        scanf("%d %d", &steps, &quit);
        if(quit)
            break;
        while(steps){
            printf("step: %d \n", steps);
            if(steps > 0){
                rotateMotor(motor, 1, 1);
                steps--;
            }
            else{
                rotateMotor(motor, 1, -1);
                steps++;          
            }
            printf("\n");
        }
    }
    motor->degrees = 0.0;
    printf("calibration finished\n");
}
