#ifndef WHEELPAIR_H
#define WHEELPAIR_H

struct WheelPair{
    unsigned pin1;
    unsigned pin2;
    unsigned pwm;
    unsigned direction; 
    unsigned dutyCycle; //range 0 to 255 (speed control)
};

struct WheelPair* setUpWheelPair(unsigned pin1, unsigned pin2, unsigned pwm);
void setDirectionFoward(struct WheelPair* wheels);
void setDirectionReverse(struct WheelPair* wheels);
void setDirectionStop(struct WheelPair* wheels);
void setSpeed(struct WheelPair* wheels, unsigned dutyCycle);

#endif