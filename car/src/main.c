#include "vehicle.h"
#include <stdio.h>
#include <stdlib.h>
#include <pigpio.h>
#include <math.h>
#include <curl/curl.h>



void get_page(const char* url, const char* file_name)
{
  CURL* easyhandle = curl_easy_init();

  curl_easy_setopt( easyhandle, CURLOPT_URL, url ) ;

  FILE* file = fopen( file_name, "w");

  curl_easy_setopt( easyhandle, CURLOPT_WRITEDATA, file) ;

  curl_easy_perform( easyhandle );

  curl_easy_cleanup( easyhandle );

  fclose(file);

}

int main()
{
    // get_page( "https://git.heroku.com/simpleserver01.git", "index.html" ) ;

    if (gpioInitialise() < 0)
    {
        perror("failed to init pigpio");
        exit(1);
    }
    unsigned right[3] = {14,15,12};
    unsigned left[3] = {23,25,13};
    struct Vehicle* vehicle = setUpVehicle(left, right, NULL);
    double x;
    double y;
    double dist;
    double orientation;
    int flag = 0;
    while(1){
      printf("enter move instructions: ");
      scanf("%lf %lf %lf %d", &x,&y, &orientation, &flag);
      if(flag){
        break;
      }
      dist = sqrt( pow(x - vehicle->x, 2) + pow(y - vehicle->y, 2)  );
      move(vehicle, dist, orientation , 1);
      stop(vehicle);
    }
    
    cleanUpVehicle(vehicle);
    gpioTerminate();

    return 0;
}
