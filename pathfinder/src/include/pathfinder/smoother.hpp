#ifndef SMOOTH_H
#define SMOOTH
#include "../vehicle.hpp"
#include "../grid.hpp"
//http://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf
//Implemented with iterative Gradient Descent
class Smoother
{
    float tolerance = .0001;
    float maxIterations = 2000;
    float alpha = .2;
    float eps = .000001;

    float weightPath = 0;
    float weightObst = 5;
    float weightVoro = .2;
    float weightCurve = 4;
    float weightSmooth = 4;

    float obstDMax = 5;
    float voroDMax = 20;
    float kMax = .9;
    Grid* grid;
public:
    Smoother(Grid* grid);
    vector<VehicleState> smooth(vector<VehicleState> path);
private:
    Coordinates2D orthogonalVector(Coordinates2D a, Coordinates2D b);
};











#endif