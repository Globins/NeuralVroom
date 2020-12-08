#ifndef VEHICLE_H
#define VEHICLE_H

#include "wheelPair.h"
#include "obstacleGrid.h"

#define DUTY 155
#define SPEED 29.0
#define TURN_CONST 80


struct Vehicle{
    struct WheelPair* leftWheels;
    struct WheelPair* rightWheels;
    // struct ObstacleGrid* obstacleGrid;
    double orientation;
    double x;
    double y;
    // struct tofSensor* frontTOF;
    // struct tofSensor* backTOF;
};

struct Vehicle* setUpVehicle(unsigned* leftWheel, unsigned* rightWheel, unsigned* stepperMotor);
void move(struct Vehicle* vehicle, int dist,  double orientation, int isFoward);
void cleanUpVehicle(struct Vehicle* vehicle);
void stop(struct Vehicle* vehicle);


#endif
