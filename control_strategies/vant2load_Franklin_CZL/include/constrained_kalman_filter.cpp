#include <iostream>
#include <Eigen/Eigen>
#include "math.h"
#include "czonotope.h"

class constrained_kalman_filter
{
	public: constrained_kalman_filter()
	{ 
		
	}
	public: ~constrained_kalman_filter()
	{
		
	}
	
	
public: static void execute(int mode, Eigen::MatrixXd xhat_plus_prev, Eigen::MatrixXd P_plus_prev, Eigen::MatrixXd u_prev, std::vector<double> y, std::vector<int> I, Eigen::MatrixXd Q, Eigen::MatrixXd R, cz Xhat, Eigen::MatrixXd& xhat_plus, Eigen::MatrixXd& P_plus)
{
        
        Eigen::MatrixXd xhat_minus;
        Eigen::MatrixXd P_minus;
        
	
	if (mode == 0) // If mode = 0, performs only prediction
	{    

		// Prediction
		prediction(xhat_plus_prev,P_plus_prev,u_prev,Q,xhat_minus,P_minus);
		xhat_plus = xhat_minus;
		P_plus = P_minus;
		
	}   
	else if(mode == 1) // If mode = 1, performs only correction. FOR INITIAL CORRECTION
	{

		// Correction
		xhat_minus = xhat_plus_prev;
		P_minus = P_plus_prev;
                correction(xhat_minus,P_minus,y,I,R,Xhat,xhat_plus,P_plus);

	}	
	else if(mode == 2) // If mode = 2, performs prediction and correction
	{
		
        	// Prediction and correction  
		prediction(xhat_plus_prev,P_plus_prev,u_prev,Q,xhat_minus,P_minus);  
                correction(xhat_minus,P_minus,y,I,R,Xhat,xhat_plus,P_plus);		      	  
		
	}		
        
}

private: static void prediction(Eigen::MatrixXd xhat_plus_prev, Eigen::MatrixXd P_plus_prev, Eigen::MatrixXd u_prev, Eigen::MatrixXd Q, Eigen::MatrixXd& xhat_minus, Eigen::MatrixXd& P_minus)
{

	// System matrices
	Eigen::MatrixXd A(20,20);
	Eigen::MatrixXd B(20,4);
	Eigen::MatrixXd F(20,3);
	
	// With NEW viscous friction coefficients
	A << 1,0,0,6.625656870062353e-21,0.0007063199999999872,2.234514001230859e-19,3.877740577424677e-10,-2.945773018334531e-06,-7.34198587602016e-06,-7.389788194397557e-06,0.012,0,0,1.590622275570099e-23,2.825279999999949e-06,8.938056004923431e-22,4.976638016281862e-12,1.390501760384238e-06,-1.107850306988483e-07,-1.110910074816425e-07,0,1,0,-0.0007063199999999873,6.61975623239473e-21,-1.917780341634211e-20,5.920774637264283e-07,-3.52818637435953e-10,2.002664893512026e-07,-2.015508518337339e-07,0,0.012,0,-2.825279999999949e-06,1.588760049043056e-23,-7.671121366536846e-23,-1.435042950842284e-06,-4.987878846526941e-12,1.431057418429249e-09,-1.445786631260176e-09,0,0,1,-2.166370614553869e-19,1.173083570228195e-19,3.456954977097624e-59,-6.382373429769427e-07,-7.945133148043064e-05,-0.0002368197723130045,-0.0002367891467003826,0,0,0.012,-8.664649873770426e-22,5.13559204452838e-22,1.303351075802275e-61,-1.119681606053284e-08,-1.591018504259264e-06,-3.757183518053368e-06,-3.75541807964769e-06,0,0,0,0.9999999999999999,-5.623109098752769e-17,-3.615155060225358e-54,-0.002050402507813573,7.635137086961007e-07,-0.0005387536192651187,0.0005404282427818222,0,0,0,0.012,-2.249244157508838e-19,-1.446457490100453e-56,-1.538665392944516e-05,9.575915545048665e-09,-5.212965832212182e-06,5.22780398386418e-06,0,0,0,5.62377575477929e-17,0.9999999999999999,-1.182433486007033e-56,7.745275044150768e-07,-0.004799518851282823,-0.007901371941486511,-0.008026942640099077,0,0,0,2.249539959603871e-19,0.012,-4.756804932702198e-59,8.734897782514298e-09,-7.057485210202556e-05,-0.0001296948791546657,-0.0001309249197457341,0,0,0,-6.550542332803423e-16,-2.778103082650787e-21,0.9999999999999999,0.0003425069473529923,-3.983682797260549e-05,0.03951553248735404,-0.03948468710021982,0,0,0,-2.620207358857695e-18,-2.233589527028381e-23,0.012,2.400440103152617e-06,-2.650915014497787e-07,0.0004521820147103304,-0.0004508463553847975,0,0,0,1.000120709778178e-19,-2.496975077837175e-21,-4.179403526352378e-54,0.9964759217685851,2.597601985958444e-06,-0.001336543361246957,0.001348206155528003,0,0,0,4.000891184372776e-22,-1.08441645816677e-23,-1.672784832989659e-56,0.011958775602382,4.453403844874737e-08,-8.963232763255631e-06,9.118552238474054e-06,0,0,0,5.36124502827739e-20,2.875794090258195e-17,-9.478394297800837e-56,3.122476121274971e-06,0.9753918612587116,-0.06684176733330879,-0.06718668515438624,0,0,0,2.381755977219716e-22,1.276573307916974e-19,-3.8069938967028e-58,4.665767322817665e-08,0.01153275053817974,-0.001065839671586768,-0.001068053682460267,0,0,0,-1.638778754948461e-20,6.236004730233056e-17,2.02073858475468e-56,-0.001522822169610527,-0.04724148482618404,0.676618714827711,0.0169871360682107,0,0,0,-5.364504624993826e-23,2.845593725851836e-19,7.845881252913644e-59,-1.079393868206081e-05,-0.001000592880916677,0.006345242654222183,-0.0004764630591343101,0,0,0,2.506398029319167e-19,6.228281903244285e-17,8.575063139002979e-58,0.001526789274576304,-0.04759849364565753,0.01694439322595311,0.6769409490247619,0,0,0,1.122177515613136e-21,2.841368220999457e-19,5.783784759397682e-61,1.091665006208604e-05,-0.001002987370041731,-0.0004766689416859331,0.006357831823079372,0,0,0,2.207937024416637e-18,0.1177199999999979,3.724190002051431e-17,9.833023498775721e-08,-0.0007158996649037336,-0.001718454777901191,-0.001731120847017013,0.9999999999999999,0,0,6.625656870062359e-21,0.0007063199999999873,2.234514001230858e-19,1.313648857596721e-09,0.0002274820201377633,-2.733379862477323e-05,-2.743041951992614e-05,0,0,0,-0.1177199999999979,2.206560456032513e-18,-3.196300569390352e-18,0.0001623616418465636,-8.765956149581983e-08,5.187585968205441e-05,-5.218607719637033e-05,0,0.9999999999999999,0,-0.0007063199999999874,6.619756232394733e-21,-1.917780341634211e-20,-0.0002385437343761214,-1.308342656473526e-09,4.087739269879847e-07,-4.12506165562497e-07,0,0,0,-3.611265804497669e-17,1.610189434613911e-17,6.41878371453752e-57,-0.0001062647464426202,-0.0105898523044863,-0.03102384268433282,-0.03103287609911908,0,0,0.9999999999999999,-2.166370614553869e-19,1.173083570228195e-19,3.456954977097623e-59,-2.080044932423405e-06,-0.0002425458428993374,-0.0005592496514218715,-0.0005593342068098806,0,0,0,4.387909246758585e-17,-9.371843563726661e-15,-6.021488317999875e-52,-0.3413062808490663,0.0001084158626165354,-0.07752711493438239,0.07777888064924832,0,0,0,0.9999999999999999,-5.623109098752768e-17,-3.615155060225356e-54,-0.003244392007325029,1.685735618859321e-06,-0.0008927720096152588,0.0008956006343160222,0,0,0,9.372728559848144e-15,5.928454349544156e-16,-1.947776378645806e-54,0.0001119458972818467,-0.7055325212051079,-1.018842565584996,-1.03724597859059,0,0,0,5.623775754779292e-17,0.9999999999999999,-1.182433486007034e-56,1.53784476017282e-06,-0.01161875577297848,-0.01911709820792907,-0.01933925922517332,0,0,0,-1.091765037426792e-13,3.339865955855235e-19,-3.478963055918521e-56,0.04787852383808208,-0.005994636565321948,5.563720880710916,-5.562207746590071,0,0,0,-6.550542332803422e-16,-2.778103082650448e-21,0.9999999999999999,0.0004705792678180712,-6.085994034104221e-05,0.07543437706367113,-0.07528621358314656,0,0,0,1.666430199108952e-17,-3.480271195646068e-19,-6.956330447296398e-52,-0.5863534095565969,0.0003523485193134143,-0.1998559191009734,0.2014603544407173,0,0,0,1.000120709778179e-19,-2.496975077837183e-21,-4.179403526352378e-54,0.9919630241972782,7.142916022884673e-06,-0.001651823233827255,0.00167624348255072,0,0,0,7.088337402238894e-18,3.810388608222961e-15,-1.567153004603272e-53,0.0004623495213680419,-3.34568361706113,-8.738050049137136,-8.792846421117776,0,0,0,5.361245028277393e-20,2.875794090258195e-17,-9.478394297800833e-56,7.782664064442982e-06,0.9279109229137176,-0.1584342134036376,-0.1589440478587663,0,0,0,-3.388147037912496e-18,7.699021663497206e-15,3.569062221565407e-54,-0.2032753395741755,-5.812479270829393,-41.68156701277555,3.806601581417362,0,0,0,-1.63877875494846e-20,6.236004730233056e-17,2.02073858475468e-56,-0.002066964644272144,-0.1441396169964681,0.1364669557427524,-0.03154841544267511,0,0,0,3.233656469854669e-17,7.694100410156985e-15,3.363866025136814e-55,0.2038822706279498,-5.864280754783493,3.803181550914355,-41.66743780197986,0,0,0,2.506398029319165e-19,6.228281903244283e-17,8.57506313900295e-58,0.002084334485041001,-0.144682978815678,-0.03158666219635126,0.1378022952797301;
	B << -1.729328068304127e-10,1.566715052278891e-10,1.755182868818321e-05,1.75851821867868e-05,-1.135173724527213e-07,1.131762663595652e-07,-1.673457774441666e-07,1.695002583220035e-07,3.046790336402814e-05,2.964554998916991e-05,0.0005402698205443253,0.0005399852200031403,0.0001602835973581164,-0.0001598016799274466,0.0005826389090255198,-0.0005842085894770591,1.116325962522345e-09,-1.811016306717044e-08,0.01885164473495473,0.01899157523097375,0.0002236563555497979,-0.000223030871107895,-0.05637049221561803,0.05613836723312176,0.00111590845083407,-0.00111255608306116,0.0006715280404787881,-0.0006924102291367638,-1.815801183213638e-06,1.672769239517585e-06,0.1535188026806026,0.1536827241297083,-0.0009999878940747694,0.0009968639697181313,0.8412574920328779,0.1054846586091876,0.0009870182371423978,-0.0009847789891139008,0.105481664308737,0.8391111962950449,-3.693603343752402e-08,3.32371792306005e-08,0.003998362549750611,0.004008126265105717,-2.949931693571237e-05,2.941067655911992e-05,-4.17014875273564e-05,4.219106274575261e-05,0.005078290535434247,0.00494064209875964,0.06448597582177343,0.06450901202189954,0.02668409482398047,-0.0266038865989765,0.07080367807002795,-0.07103447830683995,4.603872262596772e-06,-6.614823255071857e-06,2.243145253288509,2.262463317014846,0.03133476060833863,-0.03124654774759854,-7.183768915263411,7.16030529658534,0.1858679504529918,-0.1853096143989328,0.06305597451605952,-0.06560746540454335,-0.0002161022675798687,0.0001990252695342461,18.31848921406575,18.35147254087602,-0.1336678558370076,0.1332542162482928,108.0303518169917,9.70711030217716,0.1321483690730531,-0.1318445230544956,9.706211084460879,107.8277307490063;
	F << 0.0001439340901775783,6.340327409405672e-14,2.761646130495638e-08,6.311704439926612e-14,0.0001439335187696524,-1.774967322909861e-10,2.761690109775549e-08,-1.7753716350028e-10,3.103585808850341e-05,-1.571653130890702e-10,0.0002877586226146975,2.598900618749657e-07,-0.000286798905224296,1.456135687008468e-10,3.239244708282888e-05,4.173216062092782e-08,3.914219185252317e-08,-2.824627764403741e-08,-7.574905800081253e-10,0.0002873485756869907,1.728268396878263e-06,-0.0002799133633100596,7.874217869553382e-10,0.0002478232019633926,1.775958851089562e-05,-1.777456741858945e-07,0.0005524358119053403,1.779401291528518e-05,1.799229361463683e-07,0.0005520768052629371,0.0239815466037164,2.124301411975439e-11,6.507797884025108e-06,2.110000474966212e-11,0.02398136091558587,-4.621432328436854e-08,6.50800675297403e-06,-4.623156874699789e-08,0.005142145791060958,-3.969540307144336e-08,0.04793381449682953,4.321175983446021e-05,-0.04770811130670731,3.646111138256104e-08,0.004316557544020303,7.455258599518279e-06,1.025250461494096e-05,1.419025552758403e-06,-1.828384624495829e-07,0.04782713294741422,0.0002882765340371783,-0.04608552948519349,1.929147477747866e-07,0.03261723788108559,0.004081406457344634,-4.593858528572511e-05,0.06824836056915458,0.004091620199772828,4.643767995038968e-05,0.06824861736398886;	
	
	
	// Augmented system
	Eigen::MatrixXd Anu(23,23);
	Eigen::MatrixXd Bnu(23,4);
	
	Anu << A,                           F, 
	       Eigen::MatrixXd::Zero(3,20), Eigen::MatrixXd::Identity(3,3);
		  
        Bnu << B, 
	       Eigen::MatrixXd::Zero(3,4);
	       
        Eigen::MatrixXd Dw(23,23);		
        Dw << Eigen::MatrixXd::Identity(23,23);
        
        
        // Prediction
        xhat_minus = Anu*xhat_plus_prev + Bnu*u_prev;

        // Covariance (prediction)
        P_minus = Anu*P_plus_prev*Anu.transpose() + Dw*Q*Dw.transpose();
        P_minus = 0.5*(P_minus + P_minus.transpose());        
        
//	A << 0;
        
}


private: static void correction(Eigen::MatrixXd xhat_minus, Eigen::MatrixXd P_minus, std::vector<double> y, std::vector<int> I, Eigen::MatrixXd R, cz Xhat, Eigen::MatrixXd& xhat_plus, Eigen::MatrixXd& P_plus)
{

        const int nof_states = xhat_minus.rows();
        const int nof_measurements = y.size();
        const int nof_generators = Xhat.G.cols();
        const int nof_constraints = Xhat.A.rows();
        
        
	// Equilibrium output
	Eigen::MatrixXd PIeq(16,1);
	PIeq << -0.001661204266230097,1.567038189486787e-05,0.6189884034468275,-0.000131709548513303,-0.0139601530301764,1.838625741689666e-06,0,0,0,-0.006979849797580309,6.584835708791055e-05,-0.4999512749866701,0.01400528012462612,0.01380908979391114,0,0;	
        


	// System matrices
	Eigen::MatrixXd H(16,20);
	
	// With NEW viscous friction coefficients
	H << 1.000000000000002,0,0,0,0.6189884034365104,-1.567038190831635e-05,0,-0.1189884044766717,0,0,0,0,0,0,0,0,0,0,0,0,0,0.9999999999999999,0,-0.6189884034365108,0,-0.001661204266203103,0.1189884034448436,-2.187751434606082e-07,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.9999999999954489,1.567038165894966e-05,0.001661204268588177,0,-1.567038293582716e-05,-0.001661204252212605,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1.000097450847848,1.83880491670847e-06,0,-1.000097450847848,-1.838804902640196e-06,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1.838625779404502e-06,0.9999999999983099,0,1.838625719099091e-06,-0.9999999913264482,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-0.01396105997948919,-2.566916426112241e-08,0.9999999999999998,0.01396105997948914,0.0001317223833560791,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.9999025586445076,1.838446583670394e-06,0.01395969959516166,-0.9999025586445076,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.9999999913279877,-0.0001316967141875097,0,-0.9999999999999999,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-0.01395969971621931,0.0001316838814811654,0.9999025499733402,0.01395969971621931,0,0,0,0,0,0,0,0,0,9.192232728802814e-07,-0.4999512749781554,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.4999999956556586,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,6.584193990794745e-05,0.006979849798890586,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1.000000000000024,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.9999999999999195,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.9999999999999999,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.9999999999999999;		
	
	
	// Augmented system
	Eigen::MatrixXd Hnu(16,23);
	
	Hnu << H, Eigen::MatrixXd::Zero(16,3);		
	
	Eigen::MatrixXd Dv(16,16);
	Dv << Eigen::MatrixXd::Identity(16,16);
	
	
	// Turning vector<double> y into MatrixXd object, and building Ck and Dvk
	Eigen::MatrixXd ymtx(nof_measurements,1);
	Eigen::MatrixXd Hnuk(nof_measurements,23);	
	Eigen::MatrixXd Dvk(nof_measurements,16);	
	for (int i=0; i<nof_measurements; i++)
	{
		ymtx(i,0) = y.at(i);
		Hnuk.row(i) = Hnu.row(I.at(i));
		Dvk.row(i) = Dv.row(I.at(i));
	}
	
	
	
	
	// Kalman gain (constrained quadratic optimization problem)
	
	Eigen::MatrixXd A;
	Eigen::MatrixXd B;
	Eigen::MatrixXd C;
	
	A = Hnuk*P_minus*Hnuk.transpose() + Dvk*R*Dvk.transpose();
	B = Hnuk*P_minus;
	C = P_minus;
	
	
	Eigen::MatrixXd Hess(nof_states*nof_measurements + 1 + nof_generators, nof_states*nof_measurements + 1 + nof_generators);
	Eigen::MatrixXd f(nof_states*nof_measurements + 1 + nof_generators, 1);
	Eigen::MatrixXd c(1,1);
	
	Hess.setZero();
	
	
	c << C.trace();

        Eigen::Map<Eigen::VectorXd> f_map(B.data(), B.size());     
        f << -2*f_map, 0, Eigen::MatrixXd::Zero(nof_generators,1);
        
        
        Eigen::MatrixXd d(nof_measurements,1);
        Eigen::MatrixXd D(nof_states,nof_measurements*nof_states);   
        
        D.setZero();             
        
        d = ymtx - Hnuk*xhat_minus - Dvk*PIeq;
        
        for(int l = 0; l<nof_states; l++)
        {
                //Hess.block(l*nof_measurements,l*nof_measurements,nof_measurements,nof_measurements) = 2*A;
                Hess.block(l*nof_measurements,l*nof_measurements,nof_measurements,nof_measurements) = 2*(0.5*(A+A.transpose()));
                D.block(l,l*nof_measurements,1,nof_measurements) = d.transpose();
        }
        
	//std::cout << "Hess: " << std::endl << Hess << std::endl;
	//fprintf ("Failed to populate problem in: add cols.\n");
	
         //       PIeq << 0;	

        
        Eigen::MatrixXd Aineq(2 + 2*nof_generators, nof_states*nof_measurements + 1 + nof_generators);
        Eigen::MatrixXd bineq(2 + 2*nof_generators, 1);        
        
        Eigen::MatrixXd Aeq(nof_states + nof_constraints, nof_states*nof_measurements + 1 + nof_generators);
        Eigen::MatrixXd beq(nof_states + nof_constraints, 1);
        
        Eigen::MatrixXd LB;
        Eigen::MatrixXd UB;
          
        Aineq << Eigen::MatrixXd::Zero(1, nof_states*nof_measurements),                                                     1,   Eigen::MatrixXd::Zero(1,nof_generators),
                 Eigen::MatrixXd::Zero(1, nof_states*nof_measurements),                                                    -1,   Eigen::MatrixXd::Zero(1,nof_generators),    
                 Eigen::MatrixXd::Zero(nof_generators, nof_states*nof_measurements), -Eigen::MatrixXd::Ones(nof_generators,1),   Eigen::MatrixXd::Identity(nof_generators,nof_generators),
                 Eigen::MatrixXd::Zero(nof_generators, nof_states*nof_measurements), -Eigen::MatrixXd::Ones(nof_generators,1),  -Eigen::MatrixXd::Identity(nof_generators,nof_generators);
                 
                 
        bineq << 1, 0, Eigen::MatrixXd::Zero(2*nof_generators,1);
        
        Aeq <<                                                                    D,      Eigen::MatrixXd::Zero(nof_states,1), -Xhat.G,    
                Eigen::MatrixXd::Zero(nof_constraints, nof_states*nof_measurements), Eigen::MatrixXd::Zero(nof_constraints,1),  Xhat.A;
        
        beq << Xhat.c - xhat_minus,
                            Xhat.b;
                            
                            

        double objvalue;
        Eigen::MatrixXd xstar(nof_states*nof_measurements + 1 + nof_generators,1);
        int exitflag; 
        
        czonotope::cplexqpwrapper(Hess, f, Aineq, bineq, Aeq, beq, LB, UB, objvalue, xstar, exitflag); 
        if(exitflag!=0)
        {
                Aeq << 0;
        }
        
        
        
        Eigen::MatrixXd K(nof_states, nof_measurements);
        //Eigen::Map<Eigen::Matrix<double,nof_states, nof_measurements>> K_map(xstar.block(0,0,nof_states*nof_measurements,1).data(), xstar.block(0,0,nof_states*nof_measurements,1).size());
        Eigen::Map<Eigen::MatrixXd> K_map(xstar.block(0,0,nof_states*nof_measurements,1).data(), nof_measurements, nof_states);  
        K = K_map.transpose();       
        
        
        // Correction
        xhat_plus = xhat_minus + K*(ymtx - Hnuk*xhat_minus - Dvk*PIeq);

        // Covariance (correction)
        P_plus = (Eigen::MatrixXd::Identity(23,23) - K*Hnuk)*P_minus*(Eigen::MatrixXd::Identity(23,23) - K*Hnuk).transpose() + K*Dvk*R*Dvk.transpose()*K.transpose();
        P_plus = 0.5*(P_plus + P_plus.transpose());        
             
}
	
	
};


/*

function [xhat_plus, P_plus] = constrained_kalman_filter(mode, xhat_plus_prev, P_plus_prev, u_prev, y, I, Q, R, Xhat)

% Implementation of equations 5.19 from "Optimal State Estimation:
% Kalman, H-infinity and nonlinear approaches - Simon".

if(strcmp(mode,'pred')) % Only prediction
    
    [xhat_minus,P_minus] = prediction(xhat_plus_prev,P_plus_prev,u_prev,Q);
    xhat_plus = xhat_minus;
    P_plus = P_minus;

elseif(strcmp(mode,'corr')) % Only correction
    
    xhat_minus = xhat_plus_prev;
    P_minus = P_plus_prev;
    [xhat_plus,P_plus] = correction(xhat_minus,P_minus,y,R,Xhat);
    
%     C = [1, 1];
%     
%     % Correction
%     Y = czonotope(y - V.c, -V.G, V.A, V.b);
%     Xhat = czonotope_intersection(Xbar,Y,C);    
    
    %S = strip_consistent_states(Xbar,V,y);
    %Xhat = szonotope_strip_intersection(Xbar,S,algorithm);    
    
elseif(strcmp(mode,'pred-corr')) % Prediction and correction

    % Prediction and correction
    [xhat_minus,P_minus] = prediction(xhat_plus_prev,P_plus_prev,u_prev,Q);
    [xhat_plus,P_plus] = correction(xhat_minus,P_minus,y,R,Xhat);

    
%     C = [1, 1];
%     
%     % Correction
%     Y = czonotope(y - V.c, -V.G, V.A, V.b);
%     Xhat = czonotope_intersection(Xbar,Y,C);       
    
else
   
    error('Invalid estimator operation mode.');
    
end


end

function [xhat_minus,P_minus] = prediction(xhat_plus_prev,P_plus_prev,u_prev,Q)


% System matrices

A = [   0, -0.5;
        1,  0.5];
     
B = [ 1;
      0];   

Dw = eye(2);


% Prediction
xhat_minus = A*xhat_plus_prev + B*u_prev;

% Covariance (prediction)
P_minus = A*P_plus_prev*A.' + Dw*Q*Dw.';
P_minus = (P_minus + P_minus.')/2;


end

function [xhat_plus,P_plus] = correction(xhat_minus,P_minus,y,R,Xhat)


nof_states = size(xhat_minus,1);
nof_measurements = size(y,1);
nof_generators = size(Xhat.G,2);
nof_constraints = size(Xhat.A,1);

% System matrices

H = [1, 1];    
Dv = 1;


% Kalman gain (constrained quadratic optimization problem)

A = H*P_minus*H.' + Dv*R*Dv.';
B = H*P_minus;
C = P_minus;

c = trace(C);
f = -2*reshape(B,numel(B),1);

Hess = [];
for l=1:nof_states
    Hess = blkdiag(Hess, A); 
end
Hess = 2*Hess;

d = y - H*xhat_minus;
D = [];
for l=1:nof_states
   D = blkdiag(D, d.');
end

f_ = [f; 0; zeros(nof_generators,1)];
Hess_ = blkdiag(Hess,0,zeros(nof_generators));


Aineq = [ zeros(1, nof_states*nof_measurements),                                    1,  zeros(1,nof_generators);
          zeros(1, nof_states*nof_measurements),                                   -1,  zeros(1,nof_generators);    
          zeros(nof_generators, nof_states*nof_measurements), -ones(nof_generators,1),      eye(nof_generators);
          zeros(nof_generators, nof_states*nof_measurements), -ones(nof_generators,1),     -eye(nof_generators)];
    
bineq = [1; 0; zeros(2*nof_generators,1)];

Aeq = [                                                    D,      zeros(nof_states,1), -Xhat.G;    
         zeros(nof_constraints, nof_states*nof_measurements), zeros(nof_constraints,1),  Xhat.A];
         

beq = [Xhat.c - xhat_minus;
                    Xhat.b];

% Quadprog options           
OPTIONS = optimset('quadprog');          
OPTIONS.Display = 'off';
           
%xstar = quadprog(Hess_,f_,Aineq,bineq,Aeq,beq,[],[],[],OPTIONS);
xstar = cplexqp(Hess_,f_,Aineq,bineq,Aeq,beq);

K = reshape(xstar(1:nof_states*nof_measurements),nof_states,nof_measurements);

%         b = [1; 0; zeros(2*nof_generators,1)];
%         Aeq = [zeros(nof_constraints,1), Z.A];
%         beq = Z.b;
      

% Kalman gain
%K = P_minus*C.'/(C*P_minus*C.' + R);
%K = P_minus*H.'/(H*P_minus*H.' + R);
%K = P_minus*Hk.'/(Hk*P_minus*Hk.' + Rk);
%K = P_minus*H.'/(H*P_minus*H.' + Dv*R*Dv.');

% Correction
%xhat_plus = xhat_minus + K*(y - C*xhat_minus);
%xhat_plus = xhat_minus + K*(y - Hk*xhat_minus);
xhat_plus = xhat_minus + K*(y - H*xhat_minus);

% Covariance (correction)
%P_plus = (eye(23) - K*C)*P_minus*(eye(23) - K*C).' + K*R*K.';
%P_plus = (eye(23) - K*Hk)*P_minus*(eye(23) - K*Hk).' + K*Rk*K.';
P_plus = (eye(2) - K*H)*P_minus*(eye(2) - K*H).' + K*Dv*R*Dv.'*K.';
P_plus = (P_plus + P_plus.')/2;


end

% % %Q = 0.001*eye(20);
% % %Q = 10*eye(20);
% % %Q = diag([0.001*ones(3,1);0.001*ones(7,1); 10*ones(10,1)]);
% % %Q = zeros(20);
% % %Q = 0.00000001*eye(20);
% % %Q = 0.00000001*eye(23);
% % %Q = 0.00001*eye(20);
% % Q = 0.00001*eye(23);
% % %R = 1000*eye(10);
% % %R = diag([0.25*ones(3,1); zeros(7,1)]);
% % %R = diag([0.01*ones(3,1); zeros(7,1)]);
% % %R = diag([((0.05/3)^2)*ones(3,1); zeros(7,1)]); % Standard deviation = 0.05/3 m
% % R = diag([((0.05)^2)*ones(3,1); zeros(7,1)]); % Standard deviation = 0.05 m
% % %R =  0.25*eye(10);
% % %R = zeros(10);
% 
% global lineardisc;
% %global lineardiscDISTURB;
% 
% % A = lineardisc.A;
% % B = lineardisc.B;
% % C = lineardisc.C;
% 
% A = [lineardisc.A,  lineardisc.F;                   % A_nu
%       zeros(3,20),  eye(3)];
%   
% B = [lineardisc.B; zeros(3,4)];                     % B_nu
% 
% H = [eye(10),   zeros(10,10),   zeros(10,3)];       % H_nu
% 
% %A = lineardiscDISTURB.A;
% %B = lineardiscDISTURB.B;
% %C = lineardiscDISTURB.C;
% 
% % %xhat_plus_prev = in(1:20);
% % xhat_plus_prev = in(1:23);
% % %P_plus_prev = reshape(in(21:420),20,20);
% % P_plus_prev = reshape(in(24:552),23,23);
% % P_plus_prev = (P_plus_prev + P_plus_prev.')/2;
% % %u_prev = in(421:424);
% % u_prev = in(553:556);
% % %y = in(425:434);
% % y = in(557:566);
% 
% nof_measurements = length(I);
% 
% % Building H[k]
% Hdk = zeros(nof_measurements,20);
% for i=1:nof_measurements
% 
%     Hdk(i,:) = lineardisc.H(I(i),:); % Lines of H[k] according to i \in I
%     
% end	
% Hk = [Hdk, zeros(nof_measurements,3)];
% 
% % Building R[k]
% Rk = zeros(nof_measurements,nof_measurements);
% for i=1:nof_measurements
% 
%     Rk(i,i) = R(I(i),I(i)); % Diagonal elements of R[k] according to available measurements i \in I
%     
% end	
% Hk = [Hdk, zeros(nof_measurements,3)];
% 
% 
% % Prediction
% xhat_minus = A*xhat_plus_prev + B*u_prev;
% 
% % Covariance (prediction)
% P_minus = A*P_plus_prev*A.' + Q;
% 
% % Kalman gain
% %K = P_minus*C.'/(C*P_minus*C.' + R);
% %K = P_minus*H.'/(H*P_minus*H.' + R);
% K = P_minus*Hk.'/(Hk*P_minus*Hk.' + Rk);
% 
% % Correction
% %xhat_plus = xhat_minus + K*(y - C*xhat_minus);
% xhat_plus = xhat_minus + K*(y - Hk*xhat_minus);
% 
% % Covariance (correction)
% %P_plus = (eye(23) - K*C)*P_minus*(eye(23) - K*C).' + K*R*K.';
% P_plus = (eye(23) - K*Hk)*P_minus*(eye(23) - K*Hk).' + K*Rk*K.';
% 
% 
% %out = [xhat_plus; reshape(P_plus,400,1)];
% %out = [xhat_plus; reshape(P_plus,529,1)];
% 
% 

% end
*/
