#ifndef VEHICLE_H
#define VEHICLE_H

#include "wheelPair.h"
#include "obstacleGrid.h"

#define DUTY 105
#define SPEED 29.0
#define TURN_CONST 80


struct Vehicle{
    struct WheelPair* leftWheels;
    struct WheelPair* rightWheels;
    double orientation;
    double x;
    double y;
};

struct Vehicle* setUpVehicle(unsigned* leftWheel, unsigned* rightWheel, unsigned* stepperMotor);
void move(struct Vehicle* vehicle, int dist,  double orientation, int isFoward);
void cleanUpVehicle(struct Vehicle* vehicle);
void stop(struct Vehicle* vehicle);


#endif
