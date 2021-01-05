#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "obstacleGrid.h"


struct ObstacleGrid* setUpObstacleGrid(int orientation, unsigned* stepperMotor){
    struct ObstacleGrid* obstacleGrid = malloc(sizeof(struct ObstacleGrid));
    obstacleGrid->stepperMotor = setUpStepperMotor(stepperMotor[0], stepperMotor[1], stepperMotor[2], stepperMotor[3]);
    obstacleGrid->orientation = orientation;
    return obstacleGrid;
}

void mmtoCm(double* distance){
    distance[0] = distance[0]/10.0;
    distance[1] = distance[1]/10.0;
}

void getCoordinates(struct ObstacleGrid* grid, double* dist, int i){
    //------------------------front tof sensor---------------------------
    double radians = (grid->stepperMotor->degrees)*(M_PI/180.0);
    double y = sin(radians)*dist[0];
    double x = cos(radians)*dist[0];
    grid->objectCoordinates[i][0] = (double)(-(grid->orientation)) * (x);
    grid->objectCoordinates[i][1] = (double)(grid->orientation) * (y);  
    //-----------------------back tof sensor-----------------------------
    double degrees_back = grid->stepperMotor->degrees+180;
    if(degrees_back >= 360){
        degrees_back -= 360;
    }
    double radians_back = degrees_back*(M_PI/180.0);
    y = sin(radians)*dist[1];
    x = cos(radians)*dist[1];
    grid->objectCoordinates[i][2] = (double)(grid->orientation) * (x);
    grid->objectCoordinates[i][3] = (double)(-(grid->orientation)) * (y);  

}

void positiveScan(struct ObstacleGrid* grid){
    double* distance;
    for (int i=0; i<7; i++) 
	{
		tofReadDistance(distance);
        mmtoCm(distance);
        getCoordinates(grid, distance, i);
        rotateMotor(grid->stepperMotor, 14, 1);
	}
}

void negativeScan(struct ObstacleGrid* grid){
    double* distance;
    for (int i=7; i>=0; i--) 
	{
		tofReadDistance(distance);
        mmtoCm(distance);
        getCoordinates(grid, distance, i);
        rotateMotor(grid->stepperMotor, 14, -1);
	}
}

void tofReadDistance(double* distances){
    system("python3 getRanges.py");
    int front_dist = 0;
    int back_dist = 0;
    FILE *fp;
    char buff[25];
    fp = fopen("ranges.txt", "r");
    for(int i = 0; i < 5; i++){
        fgets(buff, 10,fp);
        char* front = strtok(buff, ",");
        char* back = strtok(NULL, ",");
        front_dist += atoi(front);
        back_dist += atoi(back);
    }
    distances[0] = (double) front_dist/5.0;
    distances[1] = (double) back_dist/5.0;
    fclose(fp);
}