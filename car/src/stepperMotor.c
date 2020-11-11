#include "gpioAPI.h"
#include <stdio.h>

#define A "17"
#define A1 "27"
#define B "23"
#define B1 "24"

#define HIGH 1
#define LOW 0


void setUpBoard(){
    setUpPin(A);setPinOut(A);
    setUpPin(A1);setPinOut(A1);
    setUpPin(B);setPinOut(B);
    setUpPin(B1);setPinOut(B1);
}

void clearBoard(){
    cleanUpPin(A);
    cleanUpPin(A1);
    cleanUpPin(B);
    cleanUpPin(B1);
}

void rotateMotor(int* i){
    printf("rotate: %d \n", *i);
    switch (*i)
    {
    case 0:
        writePin(A, HIGH);
        writePin(A1, LOW);
        writePin(B, LOW);
        writePin(B1, LOW);
        break;
    case 1:
        writePin(A, HIGH);
        writePin(A1, LOW);
        writePin(B, HIGH);
        writePin(B1, LOW);
        break;
    case 2:
        writePin(A, LOW);
        writePin(A1, LOW);
        writePin(B, HIGH);
        writePin(B1, LOW);
        break;
    case 3:
        writePin(A, LOW);
        writePin(A1, HIGH);
        writePin(B, HIGH);
        writePin(B1, LOW);
        break;
    case 4:
        writePin(A, LOW);
        writePin(A1, HIGH);
        writePin(B, LOW);
        writePin(B1, LOW);
        break;
    case 5:
        writePin(A, LOW);
        writePin(A1, HIGH);
        writePin(B, LOW);
        writePin(B1, HIGH);
        break;
    case 6:
        writePin(A, LOW);
        writePin(A1, LOW);
        writePin(B, LOW);
        writePin(B1, HIGH);
        break;    
    default:
        writePin(A, HIGH);
        writePin(A1, LOW);
        writePin(B, LOW);
        writePin(B1, HIGH);
        break;
    }
}


void positiveRotation(int* i){
    if(++(*i) > 7){
        *i = 0;
    }
    rotateMotor(i);
}

void negativeRotation(int* i){
    if(--(*i) < 0){
        *i = 7;
    }
    rotateMotor(i);
}

void calibrateMotor(int* i){
    int quit;
    int steps;
    printf("enter calibrations values((+/-)steps, endCalibration))\n");
    while(1)
    {
        scanf("%d %d", &steps, &quit);
        if(quit)
            break;
        while(steps){
            printf("step: %d \n", steps);
            if(steps > 0){
                positiveRotation(i);
                steps--;
            }
            else{
                negativeRotation(i);
                steps++;          
            }
            printf("\n");
            delay(250000);
        }
    }
    printf("calibration finished\n");
    
}


int main(){
    int steps;
    int quit;
    float degrees = 0.0;
    int i = 0;
    // clearBoard();
    setUpBoard();
    calibrateMotor(&i);
    printf("(+/-)steps quit: 1 step = .9 degrees\n");
    while(1){
        scanf("%d %d", &steps, &quit);
        if(quit)
            break;
        while(steps){
            if(steps < 0){
                negativeRotation(&i);
                if(degrees < 0.1)
                    degrees = 360;
                degrees = degrees - 0.9;
                steps++;
            }else{
                positiveRotation(&i);
                if(degrees > 359.5)
                    degrees = 0;
                degrees = degrees + 0.9;
                steps--;
            }
            printf("%f degrees\n", degrees);
            delay(250000);
        }
    }
    clearBoard();
    return 0;
}





