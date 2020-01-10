#include "Icontroller.hpp"
#include <iostream>
#include <Eigen/Eigen>
#include "simulator_msgs/Sensor.h"
#include "math.h"
//#include "feedfoward.cpp"
#include "routines.cpp"
#include "linear_kalman_filter.cpp"
#include "zonotopic_state_estimator.cpp"

class vant2load_Franklin : public Icontroller
{
	
	private: Eigen::VectorXd Xref;
	private: Eigen::VectorXd Erro;
	private: Eigen::VectorXd Input;
	private: Eigen::MatrixXd K;
	private: Eigen::VectorXd X;
	//private: Eigen::MatrixXd LKF_X0;
	//private: Eigen::MatrixXd LKF_P0;
	//private: Eigen::MatrixXd LKF_Q;
	//private: Eigen::MatrixXd LKF_R;
	//private: Eigen::MatrixXd ZSE_X0c;
	//private: Eigen::MatrixXd ZSE_X0G;
	//private: Eigen::MatrixXd ZSE_Wc;
	//private: Eigen::MatrixXd ZSE_WG;
	//private: Eigen::MatrixXd ZSE_Vc;
	//private: Eigen::MatrixXd ZSE_VG;
	private: double T;	
	
	public: vant2load_Franklin(): Xref(24), Erro(24), Input(4), K(4,24), X(24)
	{ 
		T = 0.012;
	}	
	public: ~vant2load_Franklin()
	{
		
	}
	public: void config()
	{               

		// SBAI gain matrix
		//K << 0.7101244468428165,6.141471260155515,7.765440795725267,-18.97385920336097,1.826066960066656,-0.09338611048322061,17.57791369983566,-0.7164683413615529,-0.03350735071807359,0.4235128078145839,0.2708588223207514,4.311055003650324,4.170146360015496,-5.072227698361692,0.3809539101204998,-0.06650607548831029,2.122392130887405,-0.02473442956879733,0.0006404185518007683,0.008513558472367464,0.4662177448319676,3.523707063368016,5.320429316782071,-0.1472247707396855,-1.571274680558446,-8.596271330525751,6.985745062499001,21.91480875426527,-1.94921538690414,0.09535891280953998,-18.42904004957552,0.4434387926540275,0.1072202681687932,-0.317249054464631,-0.9548973567439005,-5.910676814456982,4.55703288130231,5.454379896200787,-0.2518284755625079,0.06360770401465654,-2.133681658482248,-0.008445306788653999,0.001718825918710477,-0.005954099368496411,-1.2250793041699,-5.503330766582297,3.925043415306752,0.09204438852464802,0.1952075590979948,-0.03032542746108816,-0.003380038380425396,0.06292464197787898,0.3808490158409268,0.04098134842638689,-0.0437294922542443,-0.1815063858105274,0.1672299855993115,-0.004950772796723103,0.1160491338776613,-0.01901526180028523,-0.01046202265880625,0.01457474072728251,0.07364843512499895,0.02819675274652457,-0.003764578805403473,-0.008102716027547696,0.004792989032152177,0.000274744109690024,0.1320526541764119,-0.01982603318640202,0.008140109577766043,0.01296630640217303,0.1858289717940038,-0.004531750410384653,-0.001691334240977282,-0.01099980803116952,0.3650350924346544,-0.03429654457475473,0.02114487202233904,-0.1765601446988155,-0.0087900241089497,0.1676412256268605,0.1107587378532268,-0.0008316297740917086,-0.0101361709665762,-0.00430220334492462,0.07112118136129204,-0.02734833615631714,0.002775618184624419,-0.007952050965466266,0.000294409460551884,0.004683773810332413,0.1255028830715101,-0.004335263292000928,0.009748974116588649,-0.01431313055730361;
		
		K << -1.323000431092713,3.814754097193253,11.62763251296858,-14.51060269836333,-0.836959735737397,0.5148897546222578,14.85447460248879,0.4974755906446917,-7.047899966571762,10.05131094039985,-0.6190441542784982,3.01936533176857,5.20517745790634,-4.097068934232062,0.00122030545498248,-0.1242761526750772,1.924322255028726,-0.03272694589512604,-0.2974457772325438,0.3740815146936236,-1.468788155238638,1.790771837580875,10.65875862761146,0.572866277696913,-0.9314438839012262,-5.548438477322249,11.33519972358793,14.88676928093411,0.1211807916778585,-0.3413029835113952,-13.97121415390814,-0.2448368304765016,9.76479824085407,-5.682375522375405,-0.3842269066311151,-3.883578028974158,5.292704520017035,3.961683501391633,0.2057598369395902,0.2350837797245765,-1.809556636095195,-0.1052467156345451,0.3672392261625704,-0.2594445321648555,-0.9934327483207346,-3.672375491731709,10.45524098459296,-0.4561987982484454,0.053160318295129,0.01831847416215292,0.0002064332603313206,-0.0491984778367893,0.1730913056400256,0.009320485644418839,0.04830261016279114,-0.1164223719880345,0.2811058412630562,0.185313681219218,0.03901585950807417,0.01236859483364737,-0.00132220485240061,-0.01276027216937946,0.04994327496985658,-0.0001742617703074358,0.005579808157716977,-0.02077667096579222,-0.0183326464839224,0.003052050226965657,0.0310201386453351,0.01187164976206709,-0.002906359607732772,0.009457782394609827,0.05644305116513206,-0.007375939188005301,0.0007174741450496246,0.02693550594476659,0.1801593639689882,-0.01494679892403626,-0.02895760449981919,-0.1217812834374863,0.183906218096406,0.2925750245883649,0.04097305791649036,-0.006096748368744254,-0.0008814944275708132,0.007360558200411883,0.05162776490546817,-0.01526907108946667,-0.003387843506333046,-0.02136239797978148,0.003600004230595395,-0.01863922042517533,0.03415362167745017,-0.003957694539470675,-0.002306253966318808,-0.002288230750291435;
		
	}
	public: std::vector<double> execute(simulator_msgs::SensorArray arraymsg)
	{
		// Instant time k counter
		static double count = 0;

		// Integrator variables
        static double xint, x_ant = 0;
        static double yint, y_ant = 0;
        static double zint, z_ant = 0;
        static double yawint, yaw_ant = 0;		
		
		// Equilibrium values
		double equil_phi     = 0;
		double equil_theta   = 0; 
		double equil_psii    = 0;
		double equil_g1      = 1.316967145565129e-04;
		double equil_g2      = 0.013960153151247;
		double equil_aR      = 0.014005280124626;
		double equil_aL      = 0.013809089793911;
		double equil_fR      = 11.732256727680697;
		double equil_fL      = 11.767602459917352;
		double equil_tauaR   = 4.138868162580376e-07;
		double equil_tauaL   = 1.012099978192413e-05;
		
		// Getting sensor data
		//int i = 0;		
		simulator_msgs::Sensor msgstates;
		simulator_msgs::Sensor mainbody_universalsensordata;
		simulator_msgs::Sensor rightpropeller_universalsensordata;
		simulator_msgs::Sensor leftpropeller_universalsensordata;		
		simulator_msgs::Sensor load_universalsensordata;
		simulator_msgs::Sensor rightservo_universalsensordata;
		simulator_msgs::Sensor leftservo_universalsensordata;
		simulator_msgs::Sensor rodx_universalsensordata;
		simulator_msgs::Sensor rody_universalsensordata;
		
		msgstates = arraymsg.values.at(0);			
		mainbody_universalsensordata = arraymsg.values.at(1);
		rightpropeller_universalsensordata = arraymsg.values.at(2);
		leftpropeller_universalsensordata = arraymsg.values.at(3);
		load_universalsensordata = arraymsg.values.at(4);
		rightservo_universalsensordata = arraymsg.values.at(5);
		leftservo_universalsensordata = arraymsg.values.at(6);
		rodx_universalsensordata = arraymsg.values.at(7);
		rody_universalsensordata = arraymsg.values.at(8);
		
		// Reference

		double trajectoryRadius = 2;
		double trajectoryHeight = 4*trajectoryRadius;
		double trajTime = 80;
		double pi = 3.14159265358979323846;

		double xref = trajectoryRadius*cos((count*T)*2*pi/trajTime);
		double xrefdot = -trajectoryRadius*(2*pi/trajTime)*sin((count*T)*2*pi/trajTime);
		double xrefddot = -trajectoryRadius*(2*pi/trajTime)*(2*pi/trajTime)*cos((count*T)*2*pi/trajTime);

		double yref = trajectoryRadius*sin((count*T)*2*pi/trajTime);
		double yrefdot = trajectoryRadius*(2*pi/trajTime)*cos((count*T)*2*pi/trajTime);
		double yrefddot = -trajectoryRadius*(2*pi/trajTime)*(2*pi/trajTime)*sin((count*T)*2*pi/trajTime);

		double zref = trajectoryHeight+1 - trajectoryHeight*cos((count*T)*2*pi/trajTime);
		double zrefdot = trajectoryHeight*(2*pi/trajTime)*sin((count*T)*2*pi/trajTime);
		double zrefddot = trajectoryHeight*(2*pi/trajTime)*(2*pi/trajTime)*cos((count*T)*2*pi/trajTime);		
		
		Xref << 2,0,0.881,equil_phi,equil_theta,equil_psii,equil_g1,equil_g2,equil_aR,equil_aL,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
		//Xref << 2,0,1,equil_phi,equil_theta,equil_psii,equil_g1,equil_g2,equil_aR,equil_aL,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
		//Xref << xref,yref,zref,0,0,0,0.00002965,0.004885,0.004893,0.00484,xrefdot,yrefdot,zrefdot,0,0,0,0,0,0,0,0,0,0,0;
		//Xref << xref,yref,zref,equil_phi,equil_theta,equil_psii,equil_g1,equil_g2,equil_aR,equil_aL,xrefdot,yrefdot,zrefdot,0,0,0,0,0,0,0,0,0,0,0;
		
		// Generate current measurement
		// std::vector<double> y;
		// std:vector<int> I;
		// routines::generate_Measurement(count,
									   // mainbody_universalsensordata,
		                               // rightpropeller_universalsensordata,
		                               // leftpropeller_universalsensordata,		
		                               // load_universalsensordata,
		                               // rightservo_universalsensordata,
		                               // leftservo_universalsensordata,
		                               // rodx_universalsensordata,
		                               // rody_universalsensordata,	
		                               // y, I);

		//Convertendo velocidade angular
		std::vector<double> etadot = routines::wIIB2EtaDot(msgstates.values.at(13), //wIIL_x
												           msgstates.values.at(14), //wIIL_y
												           msgstates.values.at(15), //wIIL_z
												           msgstates.values.at(3),  //phi
												           msgstates.values.at(4),  //theta
												           msgstates.values.at(5)); //psii

		// Integrador Trapezoidal
		double x_atual = msgstates.values.at(0) - Xref(0);
		xint = xint + (T/2)*(x_atual + x_ant);
		x_ant = x_atual;
		double y_atual = msgstates.values.at(1) - Xref(1);
		yint = yint + (T/2)*(y_atual + y_ant);
		y_ant = y_atual;
		double z_atual = msgstates.values.at(2) - Xref(2);
		zint = zint + (T/2)*(z_atual + z_ant);
		z_ant = z_atual;
		double yaw_atual = msgstates.values.at(5) - Xref(5);
		yawint = yawint + (T/2)*(yaw_atual + yaw_ant);
		yaw_ant = yaw_atual;

		// State vector (augmented)
		X << msgstates.values.at(0),//x
			 msgstates.values.at(1),//y
			 msgstates.values.at(2),//z
			 msgstates.values.at(3),//roll
			 msgstates.values.at(4),//pitch
			 msgstates.values.at(5),//yaw
			 msgstates.values.at(8),//g1 x
			 msgstates.values.at(9),//g2 y
			 msgstates.values.at(6),//aR
			 msgstates.values.at(7),//aL
			 msgstates.values.at(10),//vx
			 msgstates.values.at(11),//vy
			 msgstates.values.at(12),//vz
			 etadot.at(0),//droll
			 etadot.at(1),//pitch
			 etadot.at(2),//yaw
			 msgstates.values.at(18),//g1dot
			 msgstates.values.at(19),//g2dot
			 msgstates.values.at(16),//aRdot
			 msgstates.values.at(17),//aLdot
			 xint,
			 yint,
			 zint,
			 yawint;


		// Tracking error
		Erro = X-Xref;
		
		// Control law
		Input = -K*Erro;

		// Null inputs (lack of gravity/free fall tests)
		// Input << 0,0,0,0;

		// Equilibrium input
		Eigen::MatrixXd ueq(4,1);
		ueq << equil_fR, equil_fL, equil_tauaR, equil_tauaL;		
		Input = ueq;

		// Equilibrium feedforward
		// Eigen::MatrixXd ueq(4,1);
		// ueq << equil_fR, equil_fL, equil_tauaR, equil_tauaL;
		// Input = Input + ueq;
				
		// Reference feedfoward
		// Eigen::MatrixXd qref(10,1);
		// qref << xref,yref,zref,equil_phi,equil_theta,equil_psii,equil_g1,equil_g2,equil_aR,equil_aL;
		// Eigen::MatrixXd qrefdot(10,1);
		// qrefdot << xrefdot,yrefdot,zrefdot,0,0,0,0,0,0,0;
		// Eigen::MatrixXd qrefddot(10,1);
		// qrefddot << xrefddot,yrefddot,zrefddot,0,0,0,0,0,0,0;
		// Eigen::MatrixXd uref = feedforward::compute(qref,qrefdot,qrefddot);
		// Input = Input + uref;

		// Feedforward
        //Input(0) = Input(0) + 12.6005;
		//Input(1) = Input(1) + 12.609;
		//Input(0) = Input(0) + equil_fR;
		//Input(1) = Input(1) + equil_fL;	
		//Input(2) = Input(2) + equil_tauaR;
		//Input(3) = Input(3) + equil_tauaL;
		// Input(0) = Input(0) + varfeedforward(0,0);
		// Input(1) = Input(1) + varfeedforward(1,0);
		// Input(2) = Input(2) + varfeedforward(2,0);
		// Input(3) = Input(3) + varfeedforward(3,0);
		count++;

		std::vector<double> out(Input.data(), Input.data() + Input.rows() * Input.cols());
		return out;
	}
	
	public: std::vector<double> Reference()
	{
		std::vector<double> out(Xref.data(), Xref.data() + Xref.rows() * Xref.cols());
		return out;
	}

	public: std::vector<double> Error()
	{
		std::vector<double> out(Erro.data(), Erro.data() + Erro.rows() * Erro.cols());	
		return out;
	}	

	public: std::vector<double> State()
	{
		std::vector<double> out(X.data(), X.data() + X.rows() * X.cols());
		return out;
	}
};


extern "C"
{
	Icontroller *create(void) {return new vant2load_Franklin;}
	void destroy(Icontroller *p) {delete p;}
}
