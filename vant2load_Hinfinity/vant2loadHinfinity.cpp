#include "Icontroller.hpp"
#include <iostream>
#include <Eigen/Eigen>
#include "include/Sensor.h"
#include "include/SensorArray.h"
#include "math.h"
#include "hinfinity.cpp"
#include "m5op.h"
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <unistd.h>
#include <ios>
#include <iostream>
#include <fstream>
#include <string>

int main()
	{
	//m5_reset_stats(0,0);
	simulator_msgs::SensorArray arraymsg;
	std::vector<double> out;
	std::vector<double> xref;
	std::vector<double> error;
	std::vector<double> x;
	int k=0;

	//std::cout << "test if main works" << std::endl;
	hinfinity* control = new hinfinity();
	//std::cout << "control initiated" << std::endl;
	control->config();
	//std::cout << "control configured" << std::endl;

	while(k<100){
	m5_reset_stats(0,0);	
	out=control->execute(arraymsg);
	m5_dump_stats(0,0);
	k++;
	//std::cout << k << " control executed" << std::endl;
	}
	//m5_dump_stats(0,0);
	return 0;
}
