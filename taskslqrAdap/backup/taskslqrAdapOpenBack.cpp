#include "Icontroller.hpp"
#include <iostream>
#include <Eigen/Eigen>
#include "include/Sensor.h"
#include "include/SensorArray.h"
#include "math.h"
//#include "hinfinity.cpp"
#include "adap.cpp"
#include "lqr.cpp"
#include "m5op.h"
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <unistd.h>
#include <ios>
#include <fstream>
#include <string>
#include <chrono>
#include <algorithm>
//#include <pthread.h>
#include <omp.h>

int countAdap=0;

void adapThread()
{

}
void lqrThread()
{


}
int main()
	{
		simulator_msgs::SensorArray arraymsg;
		std::vector<double> outA;
		std::vector<double> outL;
		std::vector<double> xref;
		std::vector<double> error;
		std::vector<double> x;

		vant3_adaptiveMixCtrl2* controlA = new vant3_adaptiveMixCtrl2();
		teste* controlL = new teste();

		controlA->config();
		controlL->config();
	//m5_reset_stats(0,0);

	#pragma omp parallel
	{
	#pragma omp sections
	{
	#pragma omp section
	while(countAdap<100){
	outL=controlL->execute(arraymsg);
	}			/* terminate the thread */
	#pragma omp section
	while(countAdap<100){
	m5_reset_stats(0,0);
	outA=controlA->execute(arraymsg);
	m5_dump_stats(0,0);
	countAdap++;
	}			/* terminate the thread */
	}
	}
	//m5_dump_stats(0,0);
	return 0;
}
