#ifndef GPIOAPI
#define GPIOAPI

//

int openWrite(char* path);

void writeTo(int fd, char* val, int length, char* path);

void setUpPin(char* pinNumber);

void cleanUpPin(char* pinNumber);

void setPinOut(char* pinNumber);

void setPinIn(char* pinNumber);

void writePin(char* pinNumber, int val);

char* readPin(char* pinNumber);

void delay(int microsec);

#endif