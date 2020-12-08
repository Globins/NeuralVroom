#ifndef GRID_H
#define GRID_H

#include "stepperMotor.h"
#include "tof.h"

struct ObstacleGrid
{
    int orientation;
    struct StepperMotor* stepperMotor;
    // struct tofSensor* tof;
    int objectCoordinates[200][2];
};


struct ObstacleGrid* setUpObstacleGrid(int orientation, unsigned* stepperMotor);
double mmtoCm(int mm);
void getCoordinates(struct ObstacleGrid* grid, double dist, int i);
void positiveScan(struct ObstacleGrid* grid);
void negativeScan(struct ObstacleGrid* grid);


#endif