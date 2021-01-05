#include "vehicle.h"
#include <stdio.h>
#include <stdlib.h>
#include <pigpio.h>
#include <string.h>
#include <math.h>
#include <curl/curl.h>



void get_page(const char* url, const char* file_name){
  CURL* easyhandle = curl_easy_init();

  curl_easy_setopt( easyhandle, CURLOPT_URL, url ) ;

  FILE* file = fopen( file_name, "w");

  curl_easy_setopt( easyhandle, CURLOPT_WRITEDATA, file) ;

  curl_easy_perform( easyhandle );

  curl_easy_cleanup( easyhandle );

  fclose(file);

}


void parseInfo(char * buff, double* x, double* y, double* orientation, int* flag){
  char * p = buff;
  p++;
  p[strlen(p)-2] = 0;
  *x = atof(strtok(p, ","))*10.0;
  *y = atof(strtok(NULL, ","))*10.0;
  *orientation = atof(strtok(NULL, ","));
  *flag = atoi(strtok(NULL, ","));
}

int main(){
  if (gpioInitialise() < 0){
      perror("failed to init pigpio");   
      exit(1);
  }
  // printf("gathering data...\n");
  // get_page( "https://neural-vroom-server.herokuapp.com/requestpath/1234/36/36", "output.txt");
  // printf("data gathered\n");

  FILE *fp;
  char buff[255];
  
  double x;
  double y;
  double orientation;
  int flag;
  double dist;
  unsigned left[3] = {14,15,12};
  unsigned right[3] = {23,25,13};
  struct Vehicle* vehicle = setUpVehicle(left, right, NULL);


  fp = fopen("output.txt", "r");
  fgets(buff, 255, (FILE*)fp);
  parseInfo(buff, &x, &y, &orientation, &flag);
  vehicle->x = x;
  vehicle->y = y;
  vehicle->orientation = orientation;
  // vehicle->orientation = 0;

  fgets(buff, 255, (FILE*)fp);
  while(!feof(fp)){
    parseInfo(buff, &x, &y, &orientation, &flag);
    dist = sqrt( pow(x - vehicle->x, 2.0) + pow(y - vehicle->y, 2.0)  );
    printf("x: %lf  y: %lf  orientation: %lf  flag: %d\n", x, y, orientation, flag);
    printf("\tdist: %f\n", dist);
    move(vehicle, dist, orientation , 1);
    // stop(vehicle);
    // time_sleep(1);
    vehicle->x = x;
    vehicle->y = y;
    vehicle->orientation = orientation;
    fgets(buff, 255, (FILE*)fp);
  }
  fclose(fp); 
    
  
  cleanUpVehicle(vehicle);
  gpioTerminate();

    return 0;
}


