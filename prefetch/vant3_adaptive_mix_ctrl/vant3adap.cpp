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
		//auto start = std::chrono::high_resolution_clock::now();
		//m5_reset_stats(0,0);
		out=control->execute(arraymsg);
	  //m5_dump_stats(0,0);
		//auto elapsed = std::chrono::high_resolution_clock::now() - start;
		//long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
		//controlData.push_back(microseconds);
		k++;
		}
		//auto elapsed = std::chrono::high_resolution_clock::now() - all;
		//long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
		//controlData.push_back(microseconds);
			//FILE *fCON = fopen("control.txt", "w");
			//	fprintf(fCON, "count;dtCon\n");
			//	fclose(fCON);
			//	for (std::vector<int>::iterator it = controlData.begin() ; it != controlData.end(); ++it){
			//		auxCount++;
			//		FILE *fCON = fopen("control.txt", "a");
			//		fprintf(fCON, "%d;%lu\n",auxCount,*it);
			//		fclose(fCON);
			//	}
			m5_dump_stats(0,0);
		return 0;
	}
