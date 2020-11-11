#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <string.h>

#include "gpioAPI.h"


int openWrite(char* path){
        int fd = open(path, O_WRONLY);
        if(fd == -1){
                char errorMessage[50] = "Unable to open ";
                strcat(errorMessage, path);
                perror(errorMessage);
                exit(1);
        }
        return fd;
}

void writeTo(int fd, char* val, int length, char* path){
        if(write(fd,val, length) != length){
                char errorMessage[50] = "Error writing to ";
                strcat(errorMessage, path);
                perror(errorMessage);
                exit(1);
        }
}

void setUpPin(char* pinNumber){
        printf("pin setup\n");
        char* path = "/sys/class/gpio/export";
        int fd = openWrite(path);
        writeTo(fd, pinNumber, 2, path);
        close(fd);
}

void cleanUpPin(char* pinNumber){
        printf("pin cleanup\n");
        char* path = "/sys/class/gpio/unexport";
        int fd = openWrite(path);
        writeTo(fd, pinNumber, 2, path);
        close(fd);
}

void setPinOut(char* pinNumber){
        char path[35];
        snprintf(path, 33, "/sys/class/gpio/gpio%s/direction", pinNumber);
        int fd = openWrite(path);
        writeTo(fd, "out", 3, path);
        close(fd);
}

void setPinIn(char* pinNumber){
        char path[35];
        snprintf(path, 33, "/sys/class/gpio/gpio%s/direction", pinNumber);
        int fd = openWrite(path);
        writeTo(fd, "in", 2, path);
        close(fd);
}

void writePin(char* pinNumber, int val){
        printf("write %d to pin %s\n", val, pinNumber);
        char path[30];
        snprintf(path, 30, "/sys/class/gpio/gpio%s/value", pinNumber);
        int fd = openWrite(path);
        writeTo(fd, val == 0? "0":"1", 1, path);
        close(fd);
}

char* readPin(char* pinNumber){
        char path[30];
        snprintf(path, 30, "/sys/class/gpio/gpio%s/value", pinNumber);
        int fd = open(path, O_RDONLY);
        char* buffer = malloc(3);
        if(fd == -1){
                perror("Unable to open");
                exit(1);
        }
        if (read(fd, buffer, 3) == -1) {
		perror("Failed to read value!\n");
		exit(1);
	}
      return buffer;  
}

void delay(int microsec){
        usleep(microsec);
}
