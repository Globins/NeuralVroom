#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "vehicle.h"
#include <pigpio.h>

struct Vehicle* setUpVehicle(unsigned leftWheel[], unsigned rightWheel[], unsigned stepperMotor[]){
    struct Vehicle* vehicle = malloc(sizeof(struct Vehicle));
    vehicle->leftWheels = setUpWheelPair(leftWheel[0], leftWheel[1], leftWheel[2]);
    vehicle->rightWheels = setUpWheelPair(rightWheel[0], rightWheel[1], rightWheel[2]);
    // vehicle->obstacleGrid = setUpObstacleGrid(1, stepperMotor);
    vehicle->orientation = 0;
    vehicle->x = 0;
    vehicle->y = 0;
    return vehicle;
}

void turnRight(struct Vehicle* vehicle, double turnAngle){
    unsigned leftSpeed = SPEED-TURN_CONST*turnAngle;
    unsigned rightSpeed = SPEED+TURN_CONST*turnAngle;
    setSpeed(vehicle->leftWheels, leftSpeed);
    setSpeed(vehicle->rightWheels, rightSpeed);
}

void turnLeft(struct Vehicle* vehicle, double turnAngle){
    unsigned leftSpeed = DUTY+TURN_CONST*turnAngle;
    unsigned rightSpeed = DUTY-TURN_CONST*turnAngle;
    setSpeed(vehicle->leftWheels, leftSpeed);
    setSpeed(vehicle->rightWheels, rightSpeed);
}

void stop(struct Vehicle* vehicle){
    setDirectionStop(vehicle->leftWheels);
    setDirectionStop(vehicle->rightWheels);
}

void move(struct Vehicle* vehicle, int dist,  double orientation, int isFoward){
    double time =(dist/SPEED);
    if(isFoward){
        setDirectionFoward(vehicle->leftWheels);
        setDirectionFoward(vehicle->rightWheels);
    }else{
        setDirectionReverse(vehicle->leftWheels);
        setDirectionReverse(vehicle->rightWheels);
    }
    if(!(vehicle->orientation-orientation)){
        setSpeed(vehicle->leftWheels, DUTY);
        setSpeed(vehicle->rightWheels, DUTY);
    }else{
        double turnAngle;
        if((7*M_PI)/4 <= vehicle->orientation){
            if(orientation <= M_PI/4){
                turnAngle = ((M_PI*2) - vehicle->orientation) + orientation;
                turnLeft(vehicle, turnAngle);
            }else if(orientation > vehicle->orientation){
                turnAngle = orientation - vehicle->orientation;
                turnLeft(vehicle, turnAngle);
                
            }else{
                turnAngle = vehicle->orientation - orientation;
                turnRight(vehicle, turnAngle);
                
            }
        } 
        else if(vehicle->orientation <= M_PI/4 ){
            if((7*M_PI)/4 <= orientation){
                turnAngle = ((M_PI*2) - orientation) + vehicle->orientation;
                turnRight(vehicle, turnAngle);
                
            }else if(orientation < vehicle->orientation){
                turnAngle = vehicle->orientation - orientation;
                turnRight(vehicle, turnAngle);
                
            }else{
                turnAngle = orientation - vehicle->orientation;
                turnLeft(vehicle, turnAngle);
                
            }
        }else{
            if(vehicle->orientation > orientation){
                turnAngle = vehicle->orientation - orientation;
                turnRight(vehicle, turnAngle);
                
            }else{
                turnAngle = orientation - vehicle->orientation;
                turnLeft(vehicle, turnAngle);
                
            }
        }
    }
    printf("move %lf\n", time);
    vehicle->orientation = orientation;
    time_sleep(time);
}

void cleanUpVehicle(struct Vehicle* vehicle){
    setDirectionStop(vehicle->leftWheels);
    setDirectionStop(vehicle->rightWheels);
    free(vehicle->leftWheels);
    free(vehicle->rightWheels);
    free(vehicle);
}

// void obstacleScan(struct Vehicle* vehicle){
    
// }
