from enum import Enum
import numpy as np
from .utils import *

class Gear(Enum):
    """Gear is a enum that returns a bool of whether the car is backwards or forwards"""
    Forward = 1
    Backward = 2

class Steer(Enum):
    """Steer is an enum that returns a bool whether the car is going foward, left or right"""
    Straight = 1
    Left = 2
    Right = 3
    
class VehicleState:
    """Vehicle state of the simulated vehicle, keeps track of the vehicles current position,
    orientation (in radians), angle of the wheels, and gear, NONDISCRETE MEASUREMENTS"""
    def __init__(self, position:tuple = (0, 0), orientation:float = 0.0, gear:Gear=Gear.Forward, wheelAngle:float = 0.0):
        self.position = position
        self.orientation = orientation
        self.wheelAngle = wheelAngle
        self.gear = gear
    def __eq__(self, other):
        return self.position == other.position and self.orientation == other.orientation
    def __ne__(self, other):
        return self.positon != other.position or self.orientation != other.orientation
    def getXPosition(self):
        return self.position[0]
    def __repr__(self):
        return 'Point(Position=%s, Orientation=%s, gear=%s)' % (self.position, np.rad2deg(self.orientation), self.gear)

class Vehicle:
    """Keeps track of vehicular statistics and holds functions for VehicleState
    ALL MEASUREMENTS IN CM, ALL MEASURMENTS ARE NONDISCRETE"""
    def __init__(self, size:tuple=(0,0), maxCarTurnAngle:float=33.75, velocity:float= 1):
        self.width = size[0]
        self.length = size[1]
        
        self.gears = [Gear.Forward, Gear.Backward]
        self.steers = [Steer.Straight, Steer.Left, Steer.Right]
        
        self.maxCarTurnAngle = maxCarTurnAngle #in Degrees
        self.turnRadius = 1#np.sin(self.maxCarTurnAngle)*self.length
        
        self.velocity = velocity

    def getNextState(self, current:VehicleState, steer:Steer, gear:Gear, dt:float):
        """returns next state based on current vehicle state"""
        length = self.velocity*dt
        x = y = angle = 0
        if(steer == Steer.Straight):
            x = length
        else:
            angle = length / self.turnRadius
            sinangle = np.sin(angle/2)
            l = 2 * sinangle * self.turnRadius
            x = l * np.cos(self.turnRadius)
            y = l * sinangle
        if(steer == Steer.Right):
            y = -y
            angle = -angle
        if(gear == Gear.Backward):
            x = -x
            angle = -angle
        correctPos = rotate((x,y), current.orientation)
        return VehicleState(tuple(np.add(current.position, correctPos)), current.orientation + angle, gear), length

