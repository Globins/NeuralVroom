#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "obstacleGrid.h"


struct ObstacleGrid* setUpObstacleGrid(int orientation, unsigned* stepperMotor){
    //TODO Update for multiple TOF sensors
    struct ObstacleGrid* obstacleGrid = malloc(sizeof(struct ObstacleGrid));
    obstacleGrid->stepperMotor = setUpStepperMotor(stepperMotor[0], stepperMotor[1], stepperMotor[2], stepperMotor[3]);
    obstacleGrid->orientation = orientation;
    // int i = tofInit(1, 0x29, 1);
    return obstacleGrid;
}

double mmtoCm(int mm){
    return (double) mm/10;
}

void getCoordinates(struct ObstacleGrid* grid, double dist, int i){
     printf("Distance = %fcm, %f degrees, i: %d\n", dist, grid->stepperMotor->degrees, i);
    float radians = (grid->stepperMotor->degrees)*(M_PI/180.0);
    float y = sin(radians)*dist;
    float x = cos(radians)*dist;
    grid->objectCoordinates[i][0] = (int) (-(grid->orientation) * (x));
    grid->objectCoordinates[i][1] = (int) (grid->orientation * (y));
    printf("\tx: %f, y: %f\n", x, y);
    printf("\tgx: %d, gy: %d\n", grid->objectCoordinates[i][0], grid->objectCoordinates[i][1]);
    
}

void positiveScan(struct ObstacleGrid* grid){
    int distance;
    double mmDist;
    for (int i=0; i<7; i++) 
	{
		distance = tofReadDistance();
        mmDist = mmtoCm(distance);
        getCoordinates(grid, mmDist, i);
        if(i != 199)
            rotateMotor(grid->stepperMotor, 14, 1);
	}
}

void negativeScan(struct ObstacleGrid* grid){
    int distance;
    double mmDist;
    for (int i=7; i>=0; i--) 
	{
		distance = tofReadDistance();
        mmDist = mmtoCm(distance);
        getCoordinates(grid, mmDist, i);
        rotateMotor(grid->stepperMotor, 14, -1);
	}
}