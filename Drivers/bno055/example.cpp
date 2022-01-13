#include <stdio.h>
#include "Jims_BNO055.h"
#include <chrono>
#include <thread>

using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono; // nanoseconds, system_clock, seconds

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Jims_BNO055 bno = Jims_BNO055(-1, 0x28);


int main(void)
{
  printf("Orientation Sensor Raw Data Test\n\n");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    printf("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!\n");
    return -1;
  }

  sleep_for(milliseconds(1000));

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  printf("Current Temperature: %d C\nn",temp);

  bno.setExtCrystalUse(true);
  sleep_for(milliseconds(500));

  for(int i=0;i<100;i++)
  {
    printf("%d Heading: %f Z: %f %d\n",i,bno.getHeading(),bno.getGyroZ(),bno.isFullyCalibrated());
  }

  while(true)
  {
    double heading =0.0;
    double gyroZ = 0.0;
    double headingAvg = 0.0;
    double gyroZAvg = 0.0;
    auto start = std::chrono::high_resolution_clock::now();
    for(int i=0;i<100;i++)
    {
      heading = bno.getHeading();
      headingAvg += heading;
      gyroZ = bno.getGyroZ();
      gyroZAvg += gyroZ;
    }
    auto elapsed = std::chrono::high_resolution_clock::now() - start;

    long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
    
    printf("micro=%lld  Heading: %f Z: %f Heading: %f Z: %f\n",microseconds,
      heading,gyroZ,headingAvg/100.0,gyroZAvg/100.0);
  }


  return 0;
}

