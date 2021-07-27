#include "Icontroller.hpp"
#include <iostream>
#include <Eigen/Eigen>
#include "include/Sensor.h"
#include "include/SensorArray.h"
#include "math.h"
#include "adap.cpp"
#include <algorithm>
#include <chrono>
#include <vector>
#include <memory>
#include <string>
#include "m5op.h"
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>



int main()
	{
	m5_reset_stats(0,0);
	std::vector<int> controlData;
	unsigned long int auxCount=0;

	simulator_msgs::SensorArray arraymsg;
	std::vector<double> out;
	std::vector<double> xref;
	std::vector<double> error;
	std::vector<double> x;
	int k=0;


//		Foo* foo1 = new Foo ();
//		std::cout << "test if main works" << std::endl;;
		vant3_adaptiveMixCtrl2* control = new vant3_adaptiveMixCtrl2();



		control->config();


//		simulator_msgs::Sensor msgstates;
//		arraymsg.header;
//		msgstates = arraymsg.values.at(0);
		//auto all = std::chrono::high_resolution_clock::now();
		while(k<100){

		//m5_reset_stats(0,0);
		out=control->execute(arraymsg);
	  //m5_dump_stats(0,0);

		k++;
		}

			m5_dump_stats(0,0);
		return 0;
	}
