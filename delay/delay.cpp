/*
Provided to you by Emlid Ltd (c) 2014.
twitter.com/emlidtech || www.emlid.com || info@emlid.com
Example: Get pressure from MS5611 barometer onboard of Navio shield for Raspberry Pi
To run this example navigate to the directory containing it and run following commands:
make
./Barometer
*/

#include <iostream>
#define _GNU_SOURCE
#include <sched.h>
#include <string>
#include <stdio.h>
#include <memory>
#include <stdint.h>
#include "m5op.h"
#include <unistd.h>
#include <sys/time.h>
//#include <Common/Util.h>
#include <pthread.h>
#include <unistd.h>
#include <vector>

int main()
{


	m5_reset_stats(0,0);
	usleep(5000);
	m5_dump_stats(0,0);

    return 0;
}
