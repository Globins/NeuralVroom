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
  *x = atof(strtok(p, ","))*25;
  *y = atof(strtok(NULL, ","))*25;
  *orientation = atof(strtok(NULL, ","));
  *flag = atoi(strtok(NULL, ","));
}

int main(){
  // get_page( "https://git.heroku.com/simpleserver01.git", "index.html" ) ;
  if (gpioInitialise() < 0){
      perror("failed to init pigpio");
      exit(1);
  }
  FILE *fp;
  char buff[255];
  
  double x;
  double y;
  double orientation;
  int flag;
  double dist;
  unsigned right[3] = {14,15,12};
  unsigned left[3] = {23,25,13};
  struct Vehicle* vehicle = setUpVehicle(left, right, NULL);


  fp = fopen("../../output.txt", "r");
  fgets(buff, 255, (FILE*)fp);
  parseInfo(buff, &x, &y, &orientation, &flag);
  vehicle->x = x;
  vehicle->y = y;
  vehicle->orientation = orientation;
  fgets(buff, 255, (FILE*)fp);
  while(!feof(fp)){
    parseInfo(buff, &x, &y, &orientation, &flag);
    dist = sqrt( pow(x - vehicle->x, 2) + pow(y - vehicle->y, 2)  );
    printf("x: %lf  y: %lf  orientation: %lf  flag: %d\n", x, y, orientation, flag);
    printf("\tdist: %f\n", dist);
    move(vehicle, dist, orientation , 1);
    stop(vehicle);
    time_sleep(1);
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
