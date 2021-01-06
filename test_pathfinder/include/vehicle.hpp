#ifndef VEHICLE_H
#define VEHICLE_H


enum Gear {Forward = 0, Backward = 1};
enum Steer {Straight = 0, Left = 1, Right = 2};

struct VehicleState
{
    float x;
    float y;
    float orientation;
    Gear gear;
    Steer steer;
    bool operator==(const VehicleState &state) const{
      return x == state.x && y == state.y && orientation == state.orientation;
  }
    bool operator!=(const VehicleState &state) const{
      return x != state.x && y != state.y && orientation != state.orientation;
  }
};

class Vehicle
{
    float width;
    float length;
    float maxTurnAngle = 33.75;
    float velocity = 0;
    int numGears = 2;
    int numSteers = 3;
    float turnRadius = 2;

public:
    Vehicle(float width, float length);
    VehicleState getNextVehicleState(VehicleState currentState, Steer steer, Gear gear, float delta_time);
};








#endif
