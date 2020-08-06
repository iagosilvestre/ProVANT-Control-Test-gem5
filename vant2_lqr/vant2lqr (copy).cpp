#include "Icontroller.hpp"
#include <iostream>
#include <Eigen/Eigen>
#include "include/Sensor.h"
#include "include/SensorArray.h"
#include "math.h"
#include "lqr.cpp"
#include "m5op.h"
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <unistd.h>
#include <ios>
#include <iostream>
#include <fstream>
#include <string>
using namespace std;
void mem_usage(double& vm_usage, double& resident_set) {
   vm_usage = 0.0;
   resident_set = 0.0;
   ifstream stat_stream("/proc/self/stat",ios_base::in); //get info from proc directory
   //create some variables to get info
   string pid, comm, state, ppid, pgrp, session, tty_nr;
   string tpgid, flags, minflt, cminflt, majflt, cmajflt;
   string utime, stime, cutime, cstime, priority, nice;
   string O, itrealvalue, starttime;
   unsigned long vsize;
   long rss;
   stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr
   >> tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt
   >> utime >> stime >> cutime >> cstime >> priority >> nice
   >> O >> itrealvalue >> starttime >> vsize >> rss; // don't care   about the rest
   stat_stream.close();
   long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024; // for x86-64 is configured to use 2MB pages
   vm_usage = vsize / 1024.0;
   resident_set = rss * page_size_kb;
}

int main()
	{
	double vm, rss;
   mem_usage(vm, rss);
   cout << "Virtual Memory: " << vm << "\nResident set size: " << rss << endl;
	simulator_msgs::SensorArray arraymsg;
	std::vector<double> out;
	std::vector<double> xref;
	std::vector<double> error;
	std::vector<double> x;
	int k=0;

//		m5_dump_stats(0,0);
//		m5_reset_stats(0,0);
//		Foo* foo1 = new Foo ();
//		std::cout << "test if main works" << std::endl;;
		teste* control = new teste();
	
//		m5_dump_stats(0,0);
//		m5_reset_stats(0,0);

		control->config();

//		m5_dump_stats(0,0);
//		simulator_msgs::Sensor msgstates;
//		arraymsg.header;
//		msgstates = arraymsg.values.at(0);

		while(k<20){
		
		//m5_reset_stats(0,0);
		out=control->execute(arraymsg);
		mem_usage(vm, rss);
  		cout << "Virtual Memory: " << vm << "\nResident set size: " << rss << endl;	
		//m5_dump_stats(0,0);

		k++;
		}
	mem_usage(vm, rss);
   cout << "Virtual Memory: " << vm << "\nResident set size: " << rss << endl;
		return 0;
	}
