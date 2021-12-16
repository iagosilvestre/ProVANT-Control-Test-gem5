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
#include <thread>

int countAdap=0;

void adapThread()
{
	simulator_msgs::SensorArray arraymsg;
	std::vector<double> outA;
	std::vector<double> xref;
	std::vector<double> error;
	std::vector<double> x;

	vant3_adaptiveMixCtrl2* controlA = new vant3_adaptiveMixCtrl2();

	controlA->config();

	while(countAdap<100){
	m5_reset_stats(0,0);
	outA=controlA->execute(arraymsg);
	m5_dump_stats(0,0);
	countAdap++;
	//std::cout << k << " control executed" << std::endl;
	}			/* terminate the thread */
}
void lqrThread()
{
	simulator_msgs::SensorArray arraymsg;
	std::vector<double> outL;
	std::vector<double> xref;
	std::vector<double> error;
	std::vector<double> x;

	teste* controlL = new teste();

	controlL->config();



	while(countAdap<100){
	//m5_reset_stats(0,0);
	outL=controlL->execute(arraymsg);
	//(0,0);

	//std::cout << k << " control executed" << std::endl;
	}			/* terminate the thread */

}
int main()
	{
	//m5_reset_stats(0,0);
	std::thread adap (adapThread);
	std::thread lqr (lqrThread);
	//std::cout << "control configured" << std::endl;
	adap.join();
	lqr.join();
	//m5_dump_stats(0,0);
	return 0;
}
