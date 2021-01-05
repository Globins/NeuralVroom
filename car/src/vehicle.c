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
    unsigned leftSpeed = DUTY+TURN_CONST*turnAngle;
    unsigned rightSpeed = DUTY-TURN_CONST*turnAngle;
    printf("\tTurn Right\n");
    printf("\t\tleftSpeed: %d\n", leftSpeed);
    printf("\t\trightSpeed: %d\n", rightSpeed);
    printf("\t\tturnAngle: %f\n", turnAngle);
    printf("\t\tTurn Diff: %d\n", (unsigned)(TURN_CONST*turnAngle));
    setSpeed(vehicle->leftWheels, leftSpeed);
    setSpeed(vehicle->rightWheels, rightSpeed);
}

void turnLeft(struct Vehicle* vehicle, double turnAngle){
    unsigned turnDifference = TURN_CONST*turnAngle;
    if(turnDifference < 30)
        turnDifference = 30;
    unsigned leftSpeed = DUTY-turnDifference;
    unsigned rightSpeed = DUTY+turnDifference;
    printf("\tTurn left\n");
    printf("\t\tleftSpeed: %d\n", leftSpeed);
    printf("\t\trightSpeed: %d\n", rightSpeed);
    printf("\t\tturnAngle: %f\n", turnAngle);
    printf("\t\tTurn Diff: %d\n", turnDifference);
    setSpeed(vehicle->leftWheels, leftSpeed);
    setSpeed(vehicle->rightWheels, rightSpeed);
}



void stop(struct Vehicle* vehicle){
    while((vehicle->leftWheels->dutyCycle > 0) || (vehicle->rightWheels->dutyCycle > 0)){
        if(vehicle->leftWheels->dutyCycle > 0){
            vehicle->leftWheels->dutyCycle--;
        }
        if(vehicle->rightWheels->dutyCycle > 0){
            vehicle->rightWheels->dutyCycle--;
        }
        // printf("%d %d\n", vehicle->leftWheels->dutyCycle, vehicle->rightWheels->dutyCycle);
        setSpeed(vehicle->leftWheels, vehicle->leftWheels->dutyCycle);
        setSpeed(vehicle->rightWheels, vehicle->rightWheels->dutyCycle);
    }
    setDirectionStop(vehicle->leftWheels);
    setDirectionStop(vehicle->rightWheels);
}

void move(struct Vehicle* vehicle, int dist,  double orientation, int isFoward){
    double time =(dist/(double)SPEED);
    if(isFoward){
        setDirectionFoward(vehicle->leftWheels);
        setDirectionFoward(vehicle->rightWheels);
    }else{
        setDirectionReverse(vehicle->leftWheels);
        setDirectionReverse(vehicle->rightWheels);
    }
    if(!(vehicle->orientation-orientation)){
        printf("\tleftSpeed: %d\n", DUTY);
        printf("\trightSpeed: %d\n", DUTY);
        setSpeed(vehicle->leftWheels, DUTY);
        setSpeed(vehicle->rightWheels, DUTY);
    }else{
        double turnAngle;

        if(vehicle->orientation >= 2.0*M_PI)
            vehicle->orientation -= 2.0*M_PI;
        if(orientation >= 2.0*M_PI)
            orientation -= 2.0*M_PI;

        if( (0 <= vehicle->orientation && vehicle->orientation < M_PI/2.0) && ( (3.0*M_PI)/2.0 < orientation && orientation < 2.0*M_PI) ){
            turnAngle = (M_PI*2.0) - orientation + vehicle->orientation;
            turnRight(vehicle, turnAngle);
        }
        else if( (0 <= orientation && orientation < M_PI/2.0) && ( (3.0*M_PI)/2.0 < vehicle->orientation && vehicle->orientation < 2.0*M_PI) ){
            turnAngle = (M_PI*2.0) - vehicle->orientation + orientation;
            turnRight(vehicle, turnAngle);
        }else{
            if(orientation > vehicle->orientation){
                turnAngle = orientation - vehicle->orientation;
                turnLeft(vehicle, turnAngle);
            }else{
                turnAngle = vehicle->orientation - orientation;
                turnRight(vehicle, turnAngle);
            }
        }
    }
    
    printf("\tmove %lf\n", time);
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
