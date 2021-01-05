#ifndef GRID_H
#define GRID_H

#include "stepperMotor.h"

struct ObstacleGrid
{
    int orientation;
    struct StepperMotor* stepperMotor;
    double objectCoordinates[10][4];
};


struct ObstacleGrid* setUpObstacleGrid(int orientation, unsigned* stepperMotor);
void mmtoCm(double* distance);
void getCoordinates(struct ObstacleGrid* grid, double* dist, int i);
void positiveScan(struct ObstacleGrid* grid);
void negativeScan(struct ObstacleGrid* grid);
void tofReadDistance();

#endif