#include "wheelPair.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <pigpio.h>

struct WheelPair* setUpWheelPair(unsigned pin1, unsigned pin2, unsigned pwm){
    gpioSetMode(pin1, PI_OUTPUT);
    gpioSetMode(pin2, PI_OUTPUT);
    gpioWrite(pin1, 0);
    gpioWrite(pin2,0);
    gpioSetMode(pwm, PI_ALT0);
    gpioPWM(pwm, 150);
    struct WheelPair* wheels = malloc(sizeof(struct WheelPair));
    wheels->pin1 = pin1;
    wheels->pin2 = pin2;
    wheels->pwm = pwm;
    wheels->direction = 0;
    wheels->dutyCycle = 0;
    return wheels;
}

void setDirectionFoward(struct WheelPair* wheels){
    if(wheels->direction == 1)
        return;
    gpioWrite(wheels->pin1, 1);
    gpioWrite(wheels->pin2, 0);
    wheels->direction = 1;   
}

void setDirectionReverse(struct WheelPair* wheels){
    
    if(wheels->direction == -1)
        return;
    gpioWrite(wheels->pin1, 0);
    gpioWrite(wheels->pin2, 1);
    wheels->direction = -1;
}

void setDirectionStop(struct WheelPair* wheels){
    if(wheels->direction == 0)
        return;
    gpioWrite(wheels->pin1, 0);
    gpioWrite(wheels->pin2, 0);
    wheels->direction = 0;
}

void setSpeed(struct WheelPair* wheels, unsigned dutyCycle){
    if(dutyCycle > 255)
        dutyCycle = 255;
    gpioPWM(wheels->pwm, dutyCycle);
    wheels->dutyCycle = dutyCycle;
}
