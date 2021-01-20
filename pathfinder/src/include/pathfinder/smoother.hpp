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

    
    float weightObst = 0;
    float weightVoro = 0;
    float weightCurve = 0;
    float weightSmooth = 0;

    float obstDMax = 5;

    Grid* grid;
public:
    Smoother(Grid* grid);
    vector<VehicleState> smooth(vector<VehicleState> path);
    Coordinates2D obstDeriv(VehicleState current);
    Coordinates2D voroDeriv(VehicleState current);
    Coordinates2D curveDeriv(VehicleState current);
    Coordinates2D smoothDeriv(VehicleState current);

};











#endif