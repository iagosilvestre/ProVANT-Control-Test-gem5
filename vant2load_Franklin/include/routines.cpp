#include <iostream>
#include <Eigen/Eigen>
#include "math.h"
#include <random>
#include "Sensor.h"

class routines
{
	public: routines()
	{ 
		
	}
	public: ~routines()
	{
		
	}

public: static void generate_Trajectory(int k, double Ts, int which_trajectory, Eigen::MatrixXd& csiref, Eigen::MatrixXd& csirefdot, Eigen::MatrixXd& csirefddot)
{
	// k = Time instant k
	double t = k*Ts;
	
	//double equil_phi     = 0;
	//double equil_theta   = 0; 
	//double equil_psii    = 0;
	//double equil_g1      = 1.316967145565129e-04;
	//double equil_g2      = 0.013960153151247;
	//double equil_aR      = 0.014005280124626;
	//double equil_aL      = 0.013809089793911;
	
	double x;
	double y;
	double z;
	double xdot;
	double ydot;
	double zdot;
	double xddot;
	double yddot;
	double zddot;
	
	// Parameters for trajectory 1
	double trajectoryRadius = 2;
	double trajectoryHeight = 4*trajectoryRadius;
	double trajTime = 40;	
	
	//double pi = M_PI;
	double pi = 3.14159265358979323846;
	
	switch ( which_trajectory ) {
		
		case 0: // Hovering
		
			x = 0;
			y = 0;
			z = 0 + 1;
			xdot = 0;
			ydot = 0;
			zdot = 0;
			xddot = 0;
			yddot = 0;
			zddot = 0;
		
		break;
		
		case 1: // Trajetoria artigo SBAI c/ Arthur
		
/* 			double trajectoryRadius = 2;
			double trajectoryHeight = 4*trajectoryRadius;
			double trajTime = 40; */
			//double pi = 3.14;

			// x0 = 2;
			// y0 = 0;
			// z0 = 1;

			x = trajectoryRadius*cos(t*2*pi/trajTime);
			xdot = -trajectoryRadius*(2*pi/trajTime)*sin(t*2*pi/trajTime);
			xddot = -trajectoryRadius*(2*pi/trajTime)*(2*pi/trajTime)*cos(t*2*pi/trajTime);

			y = trajectoryRadius*sin(t*2*pi/trajTime);
			ydot = trajectoryRadius*(2*pi/trajTime)*cos(t*2*pi/trajTime);
			yddot = -trajectoryRadius*(2*pi/trajTime)*(2*pi/trajTime)*sin(t*2*pi/trajTime);

			z = trajectoryHeight+1 - trajectoryHeight*cos(t*2*pi/trajTime);
			zdot = trajectoryHeight*(2*pi/trajTime)*sin(t*2*pi/trajTime);
			zddot = trajectoryHeight*(2*pi/trajTime)*(2*pi/trajTime)*cos(t*2*pi/trajTime);
			
		break;
		
		case 2: // Second trajectory from Rego's Master thesis

			// x0 = 0;
			// y0 = 0;
			// z0 = 1;
		
			x = 0;
			y = 0;
			z = 0;
			xdot = 0;
			ydot = 0;
			zdot = 0;
			xddot = 0;
			yddot = 0;
			zddot = 0;

			if (t<10)  // Region 1   
			{
				x = 0.01*(pow(t,2.0))*cos(pi*t/4);
				y = sin(pi*t/20)*sin(pi*t/4);
				z = 2.5 - 2.5*cos(pi*t/10) + 1;
				
				xdot = 0.02*t*cos(pi*t/4) - 0.01*(pow(t,2.0))*(pi/4)*sin(pi*t/4);
				ydot = (pi/20)*cos(pi*t/20)*sin(pi*t/4) + (pi/4)*sin(pi*t/20)*cos(pi*t/4);
				zdot = 2.5*(pi/10)*sin(pi*t/10);
				
				xddot = 0.02*cos(pi*t/4) - 0.02*t*(pi/4)*sin(pi*t/4) - ( 0.02*t*(pi/4)*sin(pi*t/4) + 0.01*(pow(t,2.0))*(pi/4)*(pi/4)*cos(pi*t/4) );
				yddot = -(pi/20)*(pi/20)*sin(pi*t/20)*sin(pi*t/4) + (pi/20)*(pi/4)*cos(pi*t/20)*cos(pi*t/4) + ((pi/4)*(pi/20)*cos(pi*t/20)*cos(pi*t/4) - (pi/4)*(pi/4)*sin(pi*t/20)*sin(pi*t/4));
				zddot = 2.5*(pi/10)*(pi/10)*cos(pi*t/10);
			}	
			else if (10<=t && t<19) // Region 2 
			{		
				x = -(pi/4)*(t-10);
				y = 1;
				z = 5 + 1;
				
				xdot = -pi/4;
				ydot = 0;
				zdot = 0;
				
				xddot = 0;
				yddot = 0;
				zddot = 0;
			}	
			else if (19<=t && t<20) // Region 3  
			{
				x = -(9*pi/4) - 0.5*sin((pi/2)*(t-19));
				y = 1.5 - 0.5*cos((pi/2)*(t-19));
				z = 5 + 1;
				
				xdot = -(pi/4)*cos((pi/2)*(t-19));
				ydot = (pi/4)*sin((pi/2)*(t-19));
				zdot = 0;
				
				xddot = (pi/4)*(pi/2)*sin((pi/2)*(t-19));
				yddot = (pi/4)*(pi/2)*cos((pi/2)*(t-19));
				zddot = 0;
			}		
			else if (20<=t && t<29) // Region 4 
			{
				x = -(9*pi/4) - 0.5;
				y = 1.5 + (pi/4)*(t-20);
				z = 5 + 1;
				
				xdot = 0;
				ydot = pi/4;
				zdot = 0;
				
				xddot = 0;
				yddot = 0;
				zddot = 0;
			}	
			else if (29<=t && t<30) // Region 5
			{
				x = -(9*pi/4) - 0.5*cos((pi/2)*(t-29));
				y = 1.5 + (9*pi/4) + 0.5*sin((pi/2)*(t-29));
				z = 5 + 1;
				
				xdot = (pi/4)*sin((pi/2)*(t-29));
				ydot = (pi/4)*cos((pi/2)*(t-29));
				zdot = 0;
				
				xddot = (pi/4)*(pi/2)*cos((pi/2)*(t-29));
				yddot = -(pi/4)*(pi/2)*sin((pi/2)*(t-29));
				zddot = 0;
			}	
			else if (30<=t && t<40) // Region 6
			{
				x = -(9*pi/4) + (pi/4)*(t-30);
				y = 2 + (9*pi/4);
				z = 5 + 1;
				
				xdot = pi/4;
				ydot = 0;
				zdot = 0;
				
				xddot = 0;
				yddot = 0;
				zddot = 0;    
			}	
			else if (40<=t && t<50) // Region 7
			{
				x = -(pi/80)*(pow(t,2.0)) + (5*pi/4)*t - (119*pi/4);
				y = 2 + (9*pi/4);
				z = 2.5 + 2.5*cos((pi/10)*(t-40)) + 1;
				
				xdot = -(pi/40)*t + (5*pi/4);
				ydot = 0;
				zdot = -2.5*(pi/10)*sin((pi/10)*(t-40));
				
				xddot = -pi/40;
				yddot = 0;
				zddot = -2.5*(pi/10)*(pi/10)*cos((pi/10)*(t-40));
			}
			else if (t>=50) // Stop
			{
				x = 4.7124;
				y = 9.0686;
				z = 0 + 1;
				
				xdot = 0;
				ydot = 0;
				zdot = 0;
				
				xddot = 0;
				yddot = 0;
				zddot = 0;
			}
		
		break;
		
		default:	

			x = 0;
			y = 0;
			z = 0;
			xdot = 0;
			ydot = 0;
			zdot = 0;
			xddot = 0;
			yddot = 0;
			zddot = 0;
		
		break;
	}
	
	//Eigen::MatrixXd Xtraj(20,1);
	//Xtraj << x, y, z, equil_phi, equil_theta, equil_psii, equil_g1, equil_g2, equil_aR, equil_aL, xdot, 
	
	//Eigen::MatrixXd qref(3,1); = {x, y, z};
	//Eigen::MatrixXd qrefdot(3,1);
	//Eigen::MatrixXd qrefddot(3,1);
	csiref << x, y, z;
	csirefdot << xdot, ydot, zdot;
	csirefddot << xddot, yddot, zddot;
	
}

public: static Eigen::MatrixXd generate_Disturbances(int k, double Ts, int which_disturbance)
{
	// k = Time instant k
	double t = k*Ts;
	
	double dtrbX = 0;
	double dtrbY = 0;
	double dtrbZ = 0;
	
	// Parameters for disturbances 1
	double finaltime = 50; // In seconds
	double dtrbMag = 0.1; // In Newtons	
	
	switch (which_disturbance){
		
		case 0: // No disturbances
		
			dtrbX = 0;
			dtrbY = 0;
			dtrbZ = 0;
			
		break;
		
		case 1: // Disturbances from Rego's Master thesis, second trajectory
		
			dtrbX = 0;
			dtrbY = 0;
			dtrbZ = 0;
			
/* 			double finaltime = 50; // In seconds
			double dtrbMag = 0.1; // In Newtons */

			//if 5<=t && t<25
			if ((finaltime/8)<=t && t<(finaltime/2 + finaltime/8))
			{
				dtrbX = dtrbMag;
				//%dtrbX = dtrbMag/2 + (dtrbMag/2)*sin(t*pi/5);
				//%dtrbX = dtrbMag + (dtrbMag/2)*sin(t*pi/5);
			}
			//%if 10<=t && t<30
			else if ((finaltime/4)<=t && t<(finaltime/2 + finaltime/4))
			{
				dtrbY = dtrbMag;
				//%dtrbY = dtrbMag + dtrbMag*sin(t*pi/5);
				//%dtrbY = 2*dtrbMag + dtrbMag*sin(t*pi/5);
			}
			//%if 15<=t && t<35
			else if ((finaltime/4 + finaltime/8)<=t && t<(finaltime/2 + finaltime/4 + finaltime/8))
			{
				dtrbZ = dtrbMag;
				//%dtrbZ = dtrbMag/2 + (dtrbMag/2)*sin(t*pi/5);
				//%dtrbZ = dtrbMag + (dtrbMag/2)*sin(t*pi/5);
				//%dtrbZ = 4*dtrbMag + 2*dtrbMag*sin(t*pi/5);
			}
			else
			{
				dtrbX = 0;
				dtrbY = 0;
				dtrbZ = 0;
			}
			
		break;
		
		default:
		
			dtrbX = 0;
			dtrbY = 0;
			dtrbZ = 0;
			
		break;		
		
	}
	
	//double dtrb[3] = {dtrbX, dtrbY, dtrbZ};
	Eigen::MatrixXd dtrb(3,1);
	dtrb << dtrbX, dtrbY, dtrbZ;
	return dtrb;
	
}

public: static double first_order_filter(double in_prev, double in)
{
	// tau = 100ms
	// A = 0.886920436717158;
	// B = 0.011307956328284;


	// tau = 500ms
	double A = 0.976285709757909;
	double B = 0.011857145121045;
	double C = 2;


	//filter_x = A*filter_x_prev + B*in;
	//out = C*filter_x;

	double out = C*(A*in_prev/C + B*in);
	
	return out;
}

//public: static double generate_Measurement(int which_sensor)
public: static void generate_Measurement(int k, double Ts, simulator_msgs::Sensor mainbody, simulator_msgs::Sensor rightpropeller,
                                                           simulator_msgs::Sensor leftpropeller, simulator_msgs::Sensor load,
                                                           simulator_msgs::Sensor rightservo, simulator_msgs::Sensor leftservo,
                                                           simulator_msgs::Sensor rodx, simulator_msgs::Sensor rody,
						           std::vector<double>& y, std::vector<int>& I)
{
	//double noise;
	//double pi = M_PI;
	double pi = 3.14159265358979323846;
	
	double dA1Bx = 0;	 
	double dA1By = 0;	 
	double dA1Bz = 0.119;
	
	// Sampling rates (in seconds)
	double T_GPS = 0.120;
	double T_Barometer = 0.012;
	double T_IMU = 0.012;
	double T_Camera = 0.024;
	double T_Servos = 0.012;
	
	// Sampling multiples
	int T_GPS_multiple = T_GPS/Ts;
	int T_Barometer_multiple = T_Barometer/Ts;
	int T_IMU_multiple = T_IMU/Ts;
	int T_Camera_multiple = T_Camera/Ts;
	int T_Servos_multiple = T_Servos/Ts;
	
	// Random number generators
	static std::default_random_engine gen00(rand());
	static std::default_random_engine gen01(rand());
	static std::default_random_engine gen02(rand());
	static std::default_random_engine gen03(rand());
	static std::default_random_engine gen04(rand());
	static std::default_random_engine gen05(rand());
	static std::default_random_engine gen06(rand());
	static std::default_random_engine gen07(rand());
	static std::default_random_engine gen08(rand());
	static std::default_random_engine gen09(rand());
	static std::default_random_engine gen10(rand());
	static std::default_random_engine gen11(rand());
	static std::default_random_engine gen12(rand());
	static std::default_random_engine gen13(rand());
	static std::default_random_engine gen14(rand());
	static std::default_random_engine gen15(rand()); // One generator for each sensor, each one with a different seed generated by rand(). Trying to ensure independency
	
	//static std::random_device rd;
    //static std::mt19937 generator(rd());
		
	std::normal_distribution<double> GPS_x(0,0.05);                                     // sigma = 0.05 m
	std::normal_distribution<double> GPS_y(0,0.05);                                     // sigma = 0.05 m
	std::normal_distribution<double> Barometer(0,0.17);                                 // sigma = 0.17 m
	std::normal_distribution<double> IMU_phi(0,(0.05*pi/180));                          // sigma = (0.05*pi/180) rad
	std::normal_distribution<double> IMU_theta(0,(0.05*pi/180));                        // sigma = (0.05*pi/180) rad
	std::normal_distribution<double> IMU_psii(0,(0.05*pi/180));                         // sigma = (0.05*pi/180) rad
	std::normal_distribution<double> IMU_p(0,0.005519215703220);                        // sigma = 0.005519215703220 rad/s
	std::normal_distribution<double> IMU_q(0,0.005519215703220);                        // sigma = 0.005519215703220 rad/s
	std::normal_distribution<double> IMU_r(0,0.005519215703220);                        // sigma = 0.005519215703220 rad/s
	std::uniform_real_distribution<double> Camera_x(-0.005,0.005);                      // resolution = 0.5 cm
	std::uniform_real_distribution<double> Camera_y(-0.005,0.005);                      // resolution = 0.5 cm
	std::uniform_real_distribution<double> Camera_z(-0.02,0.02);                        // resolution = 2 cm
	std::uniform_real_distribution<double> Servos_aR(-(0.325*pi/180),(0.325*pi/180));   // resolution = 0.325 deg
	std::uniform_real_distribution<double> Servos_aL(-(0.325*pi/180),(0.325*pi/180));   // resolution = 0.325 deg
	std::uniform_real_distribution<double> Servos_daR(-(29.09*pi/180),(29.09*pi/180));  // resolution = 29.09 deg/s
	std::uniform_real_distribution<double> Servos_daL(-(29.09*pi/180),(29.09*pi/180));  // resolution = 29.09 deg/s
	
	// Get sensor data plus noise (indexes from UniversalLinkSensor.cpp and UniversalJointSensor.cpp)
	// GPS
	double xB = (rand() % 10) + GPS_x(gen00);
	double yB = (rand() % 10) + GPS_y(gen01);
	
	// Barometer
	double zB = (rand() % 10) + Barometer(gen02);
	
	// IMU
	double phiB   = (rand() % 10) + IMU_phi(gen03);
	double thetaB = (rand() % 10) + IMU_theta(gen04);;
	double psiiB  = (rand() % 10) + IMU_psii(gen05);;
	std::vector<double> pqr = wIIL2pqr((rand() % 10), (rand() % 10), (rand() % 10),
			(rand() % 10), (rand() % 10), (rand() % 10));
	double p = pqr.at(0) + IMU_p(gen06);
	double q = pqr.at(1) + IMU_q(gen07);
	double r = pqr.at(2) + IMU_r(gen08);
	
	// Camera
	// dIA1L
//	Eigen::MatrixXd dIA1L(3,1);
//	Eigen::MatrixXd dIIA1(3,1);
//	Eigen::MatrixXd dA1A1B(3,1);
//	Eigen::MatrixXd dA1A1L(3,1);
//	Eigen::MatrixXd csiB(3,1);
//	Eigen::MatrixXd csiL(3,1);
//	Eigen::MatrixXd RIB(3,3);
//	
//	csiB << mainbody.values.at(24), mainbody.values.at(25), mainbody.values.at(26);
//	csiL << load.values.at(24), load.values.at(25), load.values.at(26);
//	dA1A1B << dA1Bx, dA1By, dA1Bz;
//	
//	RIB = rotz(mainbody.values.at(29))*roty(mainbody.values.at(28))*rotx(mainbody.values.at(27));
//	dIIA1 = csiB + RIB*(-dA1A1B);
//	dIA1L = csiL - dIIA1;
//	
//	dA1A1L = RIB.transpose()*dIA1L;
//	double dA1A1L_x = dA1A1L(0) + Camera_x(gen09);
//	double dA1A1L_y = dA1A1L(1) + Camera_y(gen10);
//	double dA1A1L_z = dA1A1L(2) + Camera_z(gen11);

        double gamma1 = (rand() % 10);
        double gamma2 = (rand() % 10);

        Eigen::MatrixXd RLB(3,3);
        RLB = rotx(-gamma1)*roty(-gamma2);
        

        double l = 0.5;
        Eigen::MatrixXd dLA1(3,1);
        dLA1 << 0, 0, l;

	Eigen::MatrixXd dA1A1L(3,1);
        dA1A1L = -RLB.transpose()*dLA1;
  	double dA1A1L_x = dA1A1L(0) + Camera_x(gen09);
	double dA1A1L_y = dA1A1L(1) + Camera_y(gen10);
	double dA1A1L_z = dA1A1L(2) + Camera_z(gen11);
	
	
	// Servos
	double aR = (rand() % 10) + Servos_aR(gen12);
	double aL = (rand() % 10) + Servos_aL(gen13);
	double aRdot = (rand() % 10) + Servos_daR(gen14);
	double aLdot = (rand() % 10) + Servos_daL(gen15);
	
	
	if(k==0) // Initial instant time, all sensors are available
	{
		y.push_back(xB);		I.push_back(0);
		y.push_back(yB);		I.push_back(1);
		y.push_back(zB);		I.push_back(2);
		y.push_back(phiB);		I.push_back(3);
		y.push_back(thetaB);		I.push_back(4);
		y.push_back(psiiB);		I.push_back(5);
		y.push_back(p);			I.push_back(6);
		y.push_back(q);			I.push_back(7);
		y.push_back(r);			I.push_back(8);
		y.push_back(dA1A1L_x);		I.push_back(9);
		y.push_back(dA1A1L_y);		I.push_back(10);
		y.push_back(dA1A1L_z);		I.push_back(11);
		y.push_back(aR);		I.push_back(12);
		y.push_back(aL);		I.push_back(13);
		y.push_back(aRdot);		I.push_back(14);
		y.push_back(aLdot);		I.push_back(15);
	}
	else 
	{
		if ((k%T_GPS_multiple)==0) // If the GPS is available
		{
			y.push_back(xB);		I.push_back(0);
			y.push_back(yB);		I.push_back(1);
		}
		
		if ((k%T_Barometer_multiple)==0) // If the Barometer is available
		{
			y.push_back(zB);		I.push_back(2);
		}		
		
		if ((k%T_IMU_multiple)==0) // If the IMU is available
		{
			y.push_back(phiB);		I.push_back(3);
			y.push_back(thetaB);	        I.push_back(4);
			y.push_back(psiiB);		I.push_back(5);
			y.push_back(p);			I.push_back(6);
			y.push_back(q);			I.push_back(7);
			y.push_back(r);			I.push_back(8);
		}		

		if ((k%T_Camera_multiple)==0) // If the Camera is available
		{
			y.push_back(dA1A1L_x);		I.push_back(9);
			y.push_back(dA1A1L_y);		I.push_back(10);
			y.push_back(dA1A1L_z);		I.push_back(11);
		}		

		if ((k%T_Servos_multiple)==0) // If the Servos sensors are available
		{
			y.push_back(aR);		I.push_back(12);
			y.push_back(aL);		I.push_back(13);
			y.push_back(aRdot);		I.push_back(14);
			y.push_back(aLdot);		I.push_back(15);
		}				
		
	}
	
}


public: static void generate_Measurement_clean(int k, double Ts, simulator_msgs::Sensor mainbody, simulator_msgs::Sensor rightpropeller,
                                                                 simulator_msgs::Sensor leftpropeller, simulator_msgs::Sensor load,
                                                                 simulator_msgs::Sensor rightservo, simulator_msgs::Sensor leftservo,
                                                                 simulator_msgs::Sensor rodx, simulator_msgs::Sensor rody,
						                 std::vector<double>& y, std::vector<int>& I)
{
	//double noise;
	//double pi = M_PI;
	double pi = 3.14159265358979323846;
	
	double dA1Bx = 0;	 
	double dA1By = 0;	 
	double dA1Bz = 0.119;
	
	// Sampling rates (in seconds)
	double T_GPS = 0.120;
	double T_Barometer = 0.012;
	double T_IMU = 0.012;
	double T_Camera = 0.024;
	double T_Servos = 0.012;
	
	// Sampling multiples
	int T_GPS_multiple = T_GPS/Ts;
	int T_Barometer_multiple = T_Barometer/Ts;
	int T_IMU_multiple = T_IMU/Ts;
	int T_Camera_multiple = T_Camera/Ts;
	int T_Servos_multiple = T_Servos/Ts;
	

	
	// Get sensor data (indexes from UniversalLinkSensor.cpp and UniversalJointSensor.cpp)
	// GPS
	double xB = (rand() % 10);
	double yB = (rand() % 10);
	
	// Barometer
	double zB = (rand() % 10);
	
	// IMU
	double phiB   = (rand() % 10);
	double thetaB = (rand() % 10);
	double psiiB  = (rand() % 10);
	std::vector<double> pqr = wIIL2pqr((rand() % 10), (rand() % 10), (rand() % 10),
			(rand() % 10), (rand() % 10), (rand() % 10));
	double p = pqr.at(0);
	double q = pqr.at(1);
	double r = pqr.at(2);
	
	// Camera
	// dIA1L
//	Eigen::MatrixXd dIA1L(3,1);
//	Eigen::MatrixXd dIIA1(3,1);
//	Eigen::MatrixXd dA1A1B(3,1);
//	Eigen::MatrixXd dA1A1L(3,1);
//	Eigen::MatrixXd csiB(3,1);
//	Eigen::MatrixXd csiL(3,1);
//	Eigen::MatrixXd RIB(3,3);
//	
//	csiB << mainbody.values.at(24), mainbody.values.at(25), mainbody.values.at(26);
//	csiL << load.values.at(24), load.values.at(25), load.values.at(26);
//	dA1A1B << dA1Bx, dA1By, dA1Bz;
//	
//	RIB = rotz(mainbody.values.at(29))*roty(mainbody.values.at(28))*rotx(mainbody.values.at(27));
//	dIIA1 = csiB + RIB*(-dA1A1B);
//	dIA1L = csiL - dIIA1;
//	
//	dA1A1L = RIB.transpose()*dIA1L;
//	double dA1A1L_x = dA1A1L(0);
//	double dA1A1L_y = dA1A1L(1);
//	double dA1A1L_z = dA1A1L(2);


        double gamma1 = (rand() % 10);
        double gamma2 = (rand() % 10);

        Eigen::MatrixXd RLB(3,3);
        RLB = rotx(-gamma1)*roty(-gamma2);
        

        double l = 0.5;
        Eigen::MatrixXd dLA1(3,1);
        dLA1 << 0, 0, l;

	Eigen::MatrixXd dA1A1L(3,1);
        dA1A1L = -RLB.transpose()*dLA1;
  	double dA1A1L_x = dA1A1L(0);
	double dA1A1L_y = dA1A1L(1);
	double dA1A1L_z = dA1A1L(2);
	
	
	// Servos
	double aR = (rand() % 10);
	double aL = (rand() % 10);
	double aRdot = (rand() % 10);
	double aLdot = (rand() % 10);
	
	
	if(k==0) // Initial instant time, all sensors are available
	{
		y.push_back(xB);		I.push_back(0);
		y.push_back(yB);		I.push_back(1);
		y.push_back(zB);		I.push_back(2);
		y.push_back(phiB);		I.push_back(3);
		y.push_back(thetaB);		I.push_back(4);
		y.push_back(psiiB);		I.push_back(5);
		y.push_back(p);			I.push_back(6);
		y.push_back(q);			I.push_back(7);
		y.push_back(r);			I.push_back(8);
		y.push_back(dA1A1L_x);		I.push_back(9);
		y.push_back(dA1A1L_y);		I.push_back(10);
		y.push_back(dA1A1L_z);		I.push_back(11);
		y.push_back(aR);		I.push_back(12);
		y.push_back(aL);		I.push_back(13);
		y.push_back(aRdot);		I.push_back(14);
		y.push_back(aLdot);		I.push_back(15);
	}
	else 
	{
		if ((k%T_GPS_multiple)==0) // If the GPS is available
		{
			y.push_back(xB);		I.push_back(0);
			y.push_back(yB);		I.push_back(1);
		}
		
		if ((k%T_Barometer_multiple)==0) // If the Barometer is available
		{
			y.push_back(zB);		I.push_back(2);
		}		
		
		if ((k%T_IMU_multiple)==0) // If the IMU is available
		{
			y.push_back(phiB);		I.push_back(3);
			y.push_back(thetaB);	        I.push_back(4);
			y.push_back(psiiB);		I.push_back(5);
			y.push_back(p);			I.push_back(6);
			y.push_back(q);			I.push_back(7);
			y.push_back(r);			I.push_back(8);
		}		

		if ((k%T_Camera_multiple)==0) // If the Camera is available
		{
			y.push_back(dA1A1L_x);		I.push_back(9);
			y.push_back(dA1A1L_y);		I.push_back(10);
			y.push_back(dA1A1L_z);		I.push_back(11);
		}		

		if ((k%T_Servos_multiple)==0) // If the Servos sensors are available
		{
			y.push_back(aR);		I.push_back(12);
			y.push_back(aL);		I.push_back(13);
			y.push_back(aRdot);		I.push_back(14);
			y.push_back(aLdot);		I.push_back(15);
		}				
		
	}
	
}

public: static std::vector<double> wIIB2EtaDot(double in_a, double in_b, double in_c, double phi, double theta, double psii)
{
	std::vector<double> out;
	out.push_back((in_a*cos(psii) + in_b*sin(psii))/cos(theta));
	out.push_back(in_b*cos(psii) - in_a*sin(psii));
	out.push_back(in_c + in_a*cos(psii)*tan(theta) + in_b*sin(psii)*tan(theta));
	return out;
}

public: static std::vector<double> pqr2EtaDot(double in_a, double in_b, double in_c, double phi, double theta, double psii)
{
	std::vector<double> out;
	out.push_back(in_a + in_c*cos(phi)*tan(theta) + in_b*sin(phi)*tan(theta));
	out.push_back(in_b*cos(phi) - in_c*sin(phi));
	out.push_back((in_c*cos(phi))/cos(theta) + (in_b*sin(phi))/cos(theta));
	return out;
}

public: static std::vector<double> wIIL2pqr(double in_a, double in_b, double in_c, double phi, double theta, double psii)
{
	// Converts wIIB to pqr
	
	std::vector<double> out;
	
	Eigen::MatrixXd RIB(3,3);
	
	RIB = rotz(psii)*roty(theta)*rotx(phi);
	
	Eigen::MatrixXd wIIB(3,1);
	
	wIIB << in_a, in_b, in_c;
	
	Eigen::MatrixXd pqr(3,1);
	
	pqr = RIB.transpose()*wIIB;
	
	out.push_back(pqr(0));
	out.push_back(pqr(1));
	out.push_back(pqr(2));
	
	//out.push_back(in_a + in_c*cos(phi)*tan(theta) + in_b*sin(phi)*tan(theta));
	//out.push_back(in_b*cos(phi) - in_c*sin(phi));
	//out.push_back((in_c*cos(phi))/cos(theta) + (in_b*sin(phi))/cos(theta));
	return out;
} 

public: static Eigen::MatrixXd rotx(double angle)
{
	// Rotation about x axis

	Eigen::MatrixXd out(3,3);
	
	out << 1, 0, 0,
	       0, cos(angle), -sin(angle),
           0, sin(angle),  cos(angle);			 
	
	return out;
}

public: static Eigen::MatrixXd roty(double angle)
{
	// Rotation about y axis

	Eigen::MatrixXd out(3,3);
	
	out <<  cos(angle), 0, sin(angle),
	        0, 1, 0,
           -sin(angle), 0, cos(angle);			 
	
	return out;
}

public: static Eigen::MatrixXd rotz(double angle)
{
	// Rotation about z axis

	Eigen::MatrixXd out(3,3);
	
	out <<  cos(angle), -sin(angle), 0,
	        sin(angle),  cos(angle), 0,
            0, 0, 1;			 
	
	return out;
}

public: static Eigen::MatrixXd get_LKFconfidencelimits(Eigen::MatrixXd Pxx)
{
	Eigen::MatrixXd Confidence(46,1);
	
	for (int i=0; i<23; i++)
	{
		Confidence(2*i) = -3*sqrt(Pxx(i,i));
		Confidence(2*i+1) =  3*sqrt(Pxx(i,i));
	}

	return Confidence;	
}

public: static Eigen::MatrixXd get_ZSEconfidencelimits(Eigen::MatrixXd XG)
{
	Eigen::MatrixXd Confidence(46,1);
	
	for (int i=0; i<23; i++)
	{
		Confidence(2*i) = -XG.block(i,0,1,XG.cols()).cwiseAbs().sum();
		Confidence(2*i+1) =  XG.block(i,0,1,XG.cols()).cwiseAbs().sum(); // Sum of absolute values of i-th line of XG
	}

	return Confidence;	
}

public: static double get_ZSEfrobnorm(Eigen::MatrixXd XG)
{
	
        double frobnorm = sqrt((XG*XG.transpose()).trace());

        return frobnorm;
	
}       


};	
