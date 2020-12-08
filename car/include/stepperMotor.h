#ifndef STEPPER
#define STEPPER

#define HIGH 1
#define LOW 0

struct StepperMotor{
    int rotationStep;
    double degrees;
    unsigned A; 
    unsigned A1;
    unsigned B; 
    unsigned B1;

};
struct StepperMotor* setUpStepperMotor(unsigned A, unsigned A1, unsigned B, unsigned B1);
void cleanUpMotor(struct StepperMotor* motor);
void rotateMotor(struct StepperMotor* motor, int rotationCount, int rotationDirection);
void calibrateMotor(struct StepperMotor* motor);


#endif