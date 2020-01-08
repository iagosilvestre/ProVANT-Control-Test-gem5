//function [Xbar,Xhat] = guaranteed_estimator_example1(Xhat_prev,y,algorithm)
//function [Xbar,Xhat] = zonotopic_state_estimator(mode,intersection_algorithm,max_zonotope_order,W,V,Xhat_prev,u,y,available_sensors)

#include <iostream>
#include <Eigen/Eigen>
#include "math.h"

class linear_kalman_filter
{
	public: linear_kalman_filter()
	{ 
		
	}
	public: ~linear_kalman_filter() 
	{
		
	}

//public: static void execute(Eigen::MatrixXd xhat_plus_prev, Eigen::MatrixXd u_prev, std::vector<double> y, std::vector<int> I, Eigen::MatrixXd Pxx_plus_prev, Eigen::MatrixXd Q, Eigen::MatrixXd R, Eigen::MatrixXd& xhat_plus, Eigen::MatrixXd& Pxx_plus, Eigen::MatrixXd& yhat_minus, Eigen::MatrixXd& Pyy_minus)
public: static void execute(Eigen::MatrixXd xhat_plus_prev, Eigen::MatrixXd u_prev, std::vector<double> y, std::vector<int> I, Eigen::MatrixXd Pxx_plus_prev, Eigen::MatrixXd Q, Eigen::MatrixXd R, Eigen::MatrixXd& xhat_plus, Eigen::MatrixXd& Pxx_plus)
{
		

	// Equilibrium outputs
	Eigen::MatrixXd PIeq(16,1);
	PIeq << -0.001661204266230097,1.567038189486787e-05,0.6189884034468275,-0.000131709548513303,-0.0139601530301764,1.838625741689666e-06,0,0,0,-0.006979849797580309,6.584835708791055e-05,-0.4999512749866701,0.01400528012462612,0.01380908979391114,0,0;

	// System matrices
	Eigen::MatrixXd A(20,20);
	Eigen::MatrixXd B(20,4);
	Eigen::MatrixXd F(20,3);
	Eigen::MatrixXd H(16,20);
	

	// Model Matrices (OLD viscous friction coefficients)
	//A << 1,0,0,-2.040374804293925e-16,0.0007063199999881863,2.139654307855388e-16,1.677364629791081e-09,-1.098658661148493e-05,-2.498317781757825e-05,-2.519942468385805e-05,0.012,0,0,-8.417941154099463e-19,2.825279999952732e-06,8.562598843798314e-19,1.601242005975512e-10,1.294813495406417e-05,-1.932289892980263e-06,-1.939512805913354e-06,0,1,0,-0.0007063199999904491,8.232940949592326e-20,1.88623821943663e-15,3.099517906000376e-06,-1.292935536833476e-09,9.693876716870242e-07,-9.746601881344724e-07,0,0.012,0,-2.825279999961802e-06,2.523642792596251e-22,7.548630313104983e-18,-1.416516298351202e-05,-1.59244915974377e-10,1.5631373744222e-08,-1.590159641927658e-08,0,0,1,5.515009902877241e-15,1.508199688736243e-17,1.2911225587423e-19,-6.263887486235375e-07,-3.339442437944485e-05,-9.182328002155424e-05,-9.167727990232351e-05,0,0,0.012,2.216688515208352e-17,1.175037008364667e-19,4.450704981517756e-22,-8.833451714705324e-08,-5.06391561527253e-06,-5.830383522268478e-06,-5.838637591893232e-06,0,0,0,0.9999999999999949,-2.279522424554841e-16,3.764548904235155e-15,-0.00203610355745686,4.139381712009717e-07,-0.0003377149221361367,0.0003386954322231747,0,0,0,0.01199999999999998,-9.375719552926276e-19,1.50750700365404e-17,-7.89529903930721e-05,3.745801601316524e-08,-9.290274060970453e-06,9.316892591264182e-06,0,0,0,6.636055914004865e-15,1.000000000000002,-4.379906726686576e-16,4.189118763510183e-07,-0.003141351766615641,-0.002742783934992212,-0.00281249278793963,0,0,0,3.044909932573477e-17,0.01200000000000001,-1.756849279898492e-18,2.959874621950415e-08,-0.0002470104714742346,-0.0001936200195018487,-0.0001962428126492743,0,0,0,1.98399852044514e-14,3.958786727775698e-15,0.9999999999999996,0.0001719629124259887,-2.362481427303445e-05,0.02057658913254932,-0.02058639923412501,0,0,0,9.184615724193891e-17,1.812590522967547e-17,0.01199999999999999,5.318778197625998e-06,-1.242797864261482e-06,0.0008643292500725051,-0.000862953436159369,0,0,0,-5.172408193052938e-15,-5.58527618978166e-17,3.721894978571586e-15,0.9965253669428077,1.327798014975014e-06,-0.001087726298867265,0.001093267989117966,0,0,0,-2.098156568527664e-17,-2.635729221320911e-19,1.494709641647812e-17,0.01171857320245931,1.679081691853671e-07,-1.284452359207974e-05,1.307355195063938e-05,0,0,0,4.247813609733422e-14,2.178036837383803e-14,-4.01134705023105e-16,1.76260382439626e-06,0.988533996619727,-0.02554707390591747,-0.02577163600298266,0,0,0,2.004808028404263e-16,1.034596631801217e-16,-1.631884340160235e-18,1.718731242311789e-07,0.01049061873002294,-0.001646495735017901,-0.001653102700546112,0,0,0,-4.801626339270842e-15,1.572827263957829e-14,5.4243085477126e-17,-0.0004201561647634117,-0.009131524706324629,0.9216148379992333,0.01611491392893182,0,0,0,-1.130200580062465e-17,9.398261564441751e-17,2.069961766313484e-19,-1.346772581769724e-05,-0.001629830498756035,0.001330231579246948,5.790422170471111e-05,0,0,0,9.79056540950484e-14,3.45983093662165e-14,6.202825501295652e-17,0.0004213342829283987,-0.009250696793667095,0.0161040709234791,0.921564188948372,0,0,0,5.512642641481328e-16,1.973187523965745e-16,2.350616017552317e-19,1.368842096880143e-05,-0.001636437724995595,5.789611337472176e-05,0.001336178118732029,0,0,0,-3.206691387158177e-14,0.117719999998032,3.562648505036742e-14,3.730557234675616e-07,-0.002440640622869894,-0.005252749562573345,-0.00530107777220095,0.9999999999999996,0,0,-2.040374804293926e-16,0.0007063199999881863,2.139654307855387e-16,3.580782804043159e-08,0.002086068613828356,-0.0003408562622315077,-0.0003423212813335496,0,0,0,-0.1177199999984078,2.055654680486759e-17,3.140530477889797e-13,0.0007862537722368871,-2.860205209563112e-07,0.0002337213845066526,-0.0002348778347452319,0,0.9999999999999996,0,-0.0007063199999904491,8.232940949592321e-20,1.88623821943663e-15,-0.002340226440874383,-3.540793750948119e-08,3.086406548696798e-06,-3.133739084399866e-06,0,0,0,9.13790599127563e-13,-3.583100365286374e-16,2.672721507804337e-17,-0.0001030745534227858,-0.004412724296479828,-0.01179420237641614,-0.01175716722956905,0,0,0.9999999999999996,5.515009902877241e-15,1.508199688736244e-17,1.2911225587423e-19,-1.483193860481095e-05,-0.0006684800162534882,-0.0004924261358229173,-0.0004932718031786567,0,0,0,-7.955697663170531e-13,-3.658731840190958e-14,6.259826381541096e-13,-0.337942553811853,5.522097475311799e-05,-0.04900361737139357,0.04913510324403749,0,0,0,0.9999999999999949,-2.27952242455484e-16,3.764548904235154e-15,-0.01372940204995748,5.168578370386444e-06,-0.0008946167956083482,0.0008966767659986146,0,0,0,9.058299393492035e-13,3.936509682720463e-13,-7.261466784527464e-14,5.657692152760978e-05,-0.4799535087949697,-0.3288486304946549,-0.3382006410163314,0,0,0,6.636055914004866e-15,1.000000000000002,-4.379906726686575e-16,4.212755003801723e-06,-0.03529893178562504,-0.01532431939911884,-0.01557496376452026,0,0,0,2.641207532681414e-12,5.376174094238153e-13,3.009469905510242e-16,0.02309594695321843,-0.003166973725869632,2.817123598043193,-2.818565207401364,0,0,0,1.983998520445141e-14,3.958786727775695e-15,0.9999999999999996,0.0007717739098103201,-0.0001782138623295741,0.08281243518473253,-0.08270050643619431,0,0,0,-8.430696204257525e-13,-6.978537997298054e-15,6.153065761713015e-13,-0.5743308774626698,0.0001778923003305875,-0.1689476162492414,0.1696936780940702,0,0,0,-5.172408193052937e-15,-5.585276189781654e-17,3.721894978571584e-15,0.9523095115088197,2.310735068120762e-05,-0.001284630901399194,0.001302933341894184,0,0,0,5.537161040654639e-12,2.807588285846411e-12,-6.478338694683873e-14,0.0002383946489510546,-1.57990442536544,-3.255412535925017,-3.285485692142776,0,0,0,4.247813609733423e-14,2.178036837383802e-14,-4.011347050231049e-16,2.410948838894007e-05,0.7980675020706058,-0.1380204315240678,-0.1386769680734872,0,0,0,-8.843425595314738e-13,1.182081249879478e-12,9.371994910155593e-15,-0.0374167401795858,-0.6939674888090763,-6.773151854695657,1.719334353244595,0,0,0,-4.801626339270856e-15,1.572827263957829e-14,5.424308547712598e-17,-0.001397575187392254,-0.1351506694541085,0.01460430896429052,0.02645638037597917,0,0,0,8.34440018118391e-12,2.877897035250543e-12,1.085251966975684e-14,0.03749533252646085,-0.7045321701609709,1.717713576745384,-6.777372479191048,0,0,0,9.790565409504836e-14,3.459830936621649e-14,6.20282550129565e-17,0.001414366707064169,-0.1358059154069367,0.02645639141363303,0.01478393531682442;
	//B << -5.075597724128965e-10,4.728523419169465e-10,3.69404005744172e-05,3.707085336315877e-05,-9.091073474159145e-07,9.063764964130174e-07,-2.526918248763945e-07,2.577473193662856e-07,3.046843002657811e-05,2.964541162684997e-05,0.0001078611399392682,0.0001080339677740663,0.000157462281693526,-0.0001569891912758619,0.0001561568157595857,-0.0001566000289593064,4.430834285469997e-08,-4.744005208439002e-08,0.003602551758894876,0.003648628832182394,0.0001134565254089457,-0.0001131284298372798,-0.01541050218799671,0.01538226733382499,0.001102281055928237,-0.001098970051725269,0.0001657499349267144,-0.0001698208258939708,-5.302225593162078e-07,5.012406912572223e-07,0.03048679077622504,0.03059855022492879,-0.0002780933830135295,0.000277250530131328,0.2046203205893211,0.0004202871570574854,0.0002758580074955485,-0.0002752053244392873,0.0004194389901285881,0.2044970854401585,-1.040314386959047e-07,9.804180957686199e-08,0.006317461688278627,0.006342437132993796,-0.0002275386799656095,0.0002268551982789999,-4.234037754019564e-05,4.318157792530773e-05,0.005077710705538437,0.00494127074780519,0.008012057116027324,0.008031890465526595,0.02585674480065093,-0.02577907104188236,0.01113803746944438,-0.01115962667550872,6.123264765806357e-06,-6.333256109836683e-06,0.2516307092825356,0.2552494195316121,0.01535628656282949,-0.01531118636245558,-1.244716921043671,1.242282144041379,0.1822940141347524,-0.1817464636677943,0.003938092050638721,-0.004193307055524278,-6.892180970062296e-05,6.669353821659653e-05,2.249467152363041,2.258106641410062,-0.02509074785341717,0.02501492395179061,18.14021058069882,-0.2068293289409435,0.02491458918465313,-0.02485534520171668,-0.2070464098030819,18.13560507263093;
	//F << 0.0001437457354001075,2.117613456547e-12,9.560406091407802e-08,2.107188382351972e-12,0.0001437289303060273,-1.415899104572709e-09,9.561062323395875e-08,-1.416075708138893e-09,3.051090065483656e-05,-6.79734197341846e-10,0.0002867366266443898,2.550152684762927e-07,-0.0002835216178417497,5.342017496192225e-10,1.360442557152496e-05,5.96633973767074e-08,9.677420657186402e-08,3.708290522992399e-08,-3.069800833468172e-09,0.0002834880619575272,1.715960408790396e-06,-0.0002596862670327839,3.122688339512173e-09,9.814229820184779e-05,3.754263962615388e-05,-2.859764804504332e-07,0.0001115442191963695,3.767736702489013e-05,2.911479407444219e-07,0.0001116958027772039,0.02393533951245774,6.455672119807491e-10,2.03127681300355e-05,6.40778924445482e-10,0.02393044891512751,-3.546620281486383e-07,2.031563845085297e-05,-3.547390135135468e-07,0.005072366637807526,-1.505425754185421e-07,0.04767959486929883,4.189513829152206e-05,-0.04700543479050505,1.191639540766302e-07,0.00179505333950121,1.133177284859766e-05,2.158902291959979e-05,5.576101128900626e-06,-6.74006592066148e-07,0.04686648675278284,0.0002839991528717244,-0.04194100191935959,6.907326548567125e-07,0.01269742628565988,0.006528760538981205,-5.437945543141847e-05,0.008978252045721637,0.006555320282751452,5.526330423135076e-05,0.008992486832820243;
	//H << 1.000000000000002,0,0,0,0.6189884034365104,-1.567038190831635e-05,0,-0.1189884044766718,0,0,0,0,0,0,0,0,0,0,0,0,0,0.9999999999999999,0,-0.6189884034365109,0,-0.001661204266203103,0.1189884034448436,-2.187751434606082e-07,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.9999999999954489,1.567038165894966e-05,0.001661204268588177,0,-1.567038293582716e-05,-0.001661204252212605,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1.000097450847848,1.83880491670847e-06,0,-1.000097450847848,-1.838804902640197e-06,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1.838625779404502e-06,0.9999999999983099,0,1.838625719099091e-06,-0.9999999913264482,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-0.01396105997948919,-2.566916426112241e-08,0.9999999999999998,0.01396105997948914,0.0001317223833560791,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.9999025586445076,1.838446583670394e-06,0.01395969959516166,-0.9999025586445076,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.9999999913279877,-0.0001316967141875097,0,-0.9999999999999999,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-0.01395969971621931,0.0001316838814811654,0.9999025499733402,0.01395969971621931,0,0,0,0,0,0,0,0,0,9.192232728802814e-07,-0.4999512749781555,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.4999999956556586,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,6.584193990794745e-05,0.006979849798890586,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1.000000000000024,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.9999999999999195,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.9999999999999999,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.9999999999999999;


	// Model Matrices (NEW viscous friction coefficients)
	A << 1,0,0,6.625656870062353e-21,0.0007063199999999872,2.234514001230859e-19,3.877740577424677e-10,-2.945773018334531e-06,-7.34198587602016e-06,-7.389788194397557e-06,0.012,0,0,1.590622275570099e-23,2.825279999999949e-06,8.938056004923431e-22,4.976638016281862e-12,1.390501760384238e-06,-1.107850306988483e-07,-1.110910074816425e-07,0,1,0,-0.0007063199999999873,6.61975623239473e-21,-1.917780341634211e-20,5.920774637264283e-07,-3.52818637435953e-10,2.002664893512026e-07,-2.015508518337339e-07,0,0.012,0,-2.825279999999949e-06,1.588760049043056e-23,-7.671121366536846e-23,-1.435042950842284e-06,-4.987878846526941e-12,1.431057418429249e-09,-1.445786631260176e-09,0,0,1,-2.166370614553869e-19,1.173083570228195e-19,3.456954977097624e-59,-6.382373429769427e-07,-7.945133148043064e-05,-0.0002368197723130045,-0.0002367891467003826,0,0,0.012,-8.664649873770426e-22,5.13559204452838e-22,1.303351075802275e-61,-1.119681606053284e-08,-1.591018504259264e-06,-3.757183518053368e-06,-3.75541807964769e-06,0,0,0,0.9999999999999999,-5.623109098752769e-17,-3.615155060225358e-54,-0.002050402507813573,7.635137086961007e-07,-0.0005387536192651187,0.0005404282427818222,0,0,0,0.012,-2.249244157508838e-19,-1.446457490100453e-56,-1.538665392944516e-05,9.575915545048665e-09,-5.212965832212182e-06,5.22780398386418e-06,0,0,0,5.62377575477929e-17,0.9999999999999999,-1.182433486007033e-56,7.745275044150768e-07,-0.004799518851282823,-0.007901371941486511,-0.008026942640099077,0,0,0,2.249539959603871e-19,0.012,-4.756804932702198e-59,8.734897782514298e-09,-7.057485210202556e-05,-0.0001296948791546657,-0.0001309249197457341,0,0,0,-6.550542332803423e-16,-2.778103082650787e-21,0.9999999999999999,0.0003425069473529923,-3.983682797260549e-05,0.03951553248735404,-0.03948468710021982,0,0,0,-2.620207358857695e-18,-2.233589527028381e-23,0.012,2.400440103152617e-06,-2.650915014497787e-07,0.0004521820147103304,-0.0004508463553847975,0,0,0,1.000120709778178e-19,-2.496975077837175e-21,-4.179403526352378e-54,0.9964759217685851,2.597601985958444e-06,-0.001336543361246957,0.001348206155528003,0,0,0,4.000891184372776e-22,-1.08441645816677e-23,-1.672784832989659e-56,0.011958775602382,4.453403844874737e-08,-8.963232763255631e-06,9.118552238474054e-06,0,0,0,5.36124502827739e-20,2.875794090258195e-17,-9.478394297800837e-56,3.122476121274971e-06,0.9753918612587116,-0.06684176733330879,-0.06718668515438624,0,0,0,2.381755977219716e-22,1.276573307916974e-19,-3.8069938967028e-58,4.665767322817665e-08,0.01153275053817974,-0.001065839671586768,-0.001068053682460267,0,0,0,-1.638778754948461e-20,6.236004730233056e-17,2.02073858475468e-56,-0.001522822169610527,-0.04724148482618404,0.676618714827711,0.0169871360682107,0,0,0,-5.364504624993826e-23,2.845593725851836e-19,7.845881252913644e-59,-1.079393868206081e-05,-0.001000592880916677,0.006345242654222183,-0.0004764630591343101,0,0,0,2.506398029319167e-19,6.228281903244285e-17,8.575063139002979e-58,0.001526789274576304,-0.04759849364565753,0.01694439322595311,0.6769409490247619,0,0,0,1.122177515613136e-21,2.841368220999457e-19,5.783784759397682e-61,1.091665006208604e-05,-0.001002987370041731,-0.0004766689416859331,0.006357831823079372,0,0,0,2.207937024416637e-18,0.1177199999999979,3.724190002051431e-17,9.833023498775721e-08,-0.0007158996649037336,-0.001718454777901191,-0.001731120847017013,0.9999999999999999,0,0,6.625656870062359e-21,0.0007063199999999873,2.234514001230858e-19,1.313648857596721e-09,0.0002274820201377633,-2.733379862477323e-05,-2.743041951992614e-05,0,0,0,-0.1177199999999979,2.206560456032513e-18,-3.196300569390352e-18,0.0001623616418465636,-8.765956149581983e-08,5.187585968205441e-05,-5.218607719637033e-05,0,0.9999999999999999,0,-0.0007063199999999874,6.619756232394733e-21,-1.917780341634211e-20,-0.0002385437343761214,-1.308342656473526e-09,4.087739269879847e-07,-4.12506165562497e-07,0,0,0,-3.611265804497669e-17,1.610189434613911e-17,6.41878371453752e-57,-0.0001062647464426202,-0.0105898523044863,-0.03102384268433282,-0.03103287609911908,0,0,0.9999999999999999,-2.166370614553869e-19,1.173083570228195e-19,3.456954977097623e-59,-2.080044932423405e-06,-0.0002425458428993374,-0.0005592496514218715,-0.0005593342068098806,0,0,0,4.387909246758585e-17,-9.371843563726661e-15,-6.021488317999875e-52,-0.3413062808490663,0.0001084158626165354,-0.07752711493438239,0.07777888064924832,0,0,0,0.9999999999999999,-5.623109098752768e-17,-3.615155060225356e-54,-0.003244392007325029,1.685735618859321e-06,-0.0008927720096152588,0.0008956006343160222,0,0,0,9.372728559848144e-15,5.928454349544156e-16,-1.947776378645806e-54,0.0001119458972818467,-0.7055325212051079,-1.018842565584996,-1.03724597859059,0,0,0,5.623775754779292e-17,0.9999999999999999,-1.182433486007034e-56,1.53784476017282e-06,-0.01161875577297848,-0.01911709820792907,-0.01933925922517332,0,0,0,-1.091765037426792e-13,3.339865955855235e-19,-3.478963055918521e-56,0.04787852383808208,-0.005994636565321948,5.563720880710916,-5.562207746590071,0,0,0,-6.550542332803422e-16,-2.778103082650448e-21,0.9999999999999999,0.0004705792678180712,-6.085994034104221e-05,0.07543437706367113,-0.07528621358314656,0,0,0,1.666430199108952e-17,-3.480271195646068e-19,-6.956330447296398e-52,-0.5863534095565969,0.0003523485193134143,-0.1998559191009734,0.2014603544407173,0,0,0,1.000120709778179e-19,-2.496975077837183e-21,-4.179403526352378e-54,0.9919630241972782,7.142916022884673e-06,-0.001651823233827255,0.00167624348255072,0,0,0,7.088337402238894e-18,3.810388608222961e-15,-1.567153004603272e-53,0.0004623495213680419,-3.34568361706113,-8.738050049137136,-8.792846421117776,0,0,0,5.361245028277393e-20,2.875794090258195e-17,-9.478394297800833e-56,7.782664064442982e-06,0.9279109229137176,-0.1584342134036376,-0.1589440478587663,0,0,0,-3.388147037912496e-18,7.699021663497206e-15,3.569062221565407e-54,-0.2032753395741755,-5.812479270829393,-41.68156701277555,3.806601581417362,0,0,0,-1.63877875494846e-20,6.236004730233056e-17,2.02073858475468e-56,-0.002066964644272144,-0.1441396169964681,0.1364669557427524,-0.03154841544267511,0,0,0,3.233656469854669e-17,7.694100410156985e-15,3.363866025136814e-55,0.2038822706279498,-5.864280754783493,3.803181550914355,-41.66743780197986,0,0,0,2.506398029319165e-19,6.228281903244283e-17,8.57506313900295e-58,0.002084334485041001,-0.144682978815678,-0.03158666219635126,0.1378022952797301;
	B << -1.729328068304127e-10,1.566715052278891e-10,1.755182868818321e-05,1.75851821867868e-05,-1.135173724527213e-07,1.131762663595652e-07,-1.673457774441666e-07,1.695002583220035e-07,3.046790336402814e-05,2.964554998916991e-05,0.0005402698205443253,0.0005399852200031403,0.0001602835973581164,-0.0001598016799274466,0.0005826389090255198,-0.0005842085894770591,1.116325962522345e-09,-1.811016306717044e-08,0.01885164473495473,0.01899157523097375,0.0002236563555497979,-0.000223030871107895,-0.05637049221561803,0.05613836723312176,0.00111590845083407,-0.00111255608306116,0.0006715280404787881,-0.0006924102291367638,-1.815801183213638e-06,1.672769239517585e-06,0.1535188026806026,0.1536827241297083,-0.0009999878940747694,0.0009968639697181313,0.8412574920328779,0.1054846586091876,0.0009870182371423978,-0.0009847789891139008,0.105481664308737,0.8391111962950449,-3.693603343752402e-08,3.32371792306005e-08,0.003998362549750611,0.004008126265105717,-2.949931693571237e-05,2.941067655911992e-05,-4.17014875273564e-05,4.219106274575261e-05,0.005078290535434247,0.00494064209875964,0.06448597582177343,0.06450901202189954,0.02668409482398047,-0.0266038865989765,0.07080367807002795,-0.07103447830683995,4.603872262596772e-06,-6.614823255071857e-06,2.243145253288509,2.262463317014846,0.03133476060833863,-0.03124654774759854,-7.183768915263411,7.16030529658534,0.1858679504529918,-0.1853096143989328,0.06305597451605952,-0.06560746540454335,-0.0002161022675798687,0.0001990252695342461,18.31848921406575,18.35147254087602,-0.1336678558370076,0.1332542162482928,108.0303518169917,9.70711030217716,0.1321483690730531,-0.1318445230544956,9.706211084460879,107.8277307490063;
	F << 0.0001439340901775783,6.340327409405672e-14,2.761646130495638e-08,6.311704439926612e-14,0.0001439335187696524,-1.774967322909861e-10,2.761690109775549e-08,-1.7753716350028e-10,3.103585808850341e-05,-1.571653130890702e-10,0.0002877586226146975,2.598900618749657e-07,-0.000286798905224296,1.456135687008468e-10,3.239244708282888e-05,4.173216062092782e-08,3.914219185252317e-08,-2.824627764403741e-08,-7.574905800081253e-10,0.0002873485756869907,1.728268396878263e-06,-0.0002799133633100596,7.874217869553382e-10,0.0002478232019633926,1.775958851089562e-05,-1.777456741858945e-07,0.0005524358119053403,1.779401291528518e-05,1.799229361463683e-07,0.0005520768052629371,0.0239815466037164,2.124301411975439e-11,6.507797884025108e-06,2.110000474966212e-11,0.02398136091558587,-4.621432328436854e-08,6.50800675297403e-06,-4.623156874699789e-08,0.005142145791060958,-3.969540307144336e-08,0.04793381449682953,4.321175983446021e-05,-0.04770811130670731,3.646111138256104e-08,0.004316557544020303,7.455258599518279e-06,1.025250461494096e-05,1.419025552758403e-06,-1.828384624495829e-07,0.04782713294741422,0.0002882765340371783,-0.04608552948519349,1.929147477747866e-07,0.03261723788108559,0.004081406457344634,-4.593858528572511e-05,0.06824836056915458,0.004091620199772828,4.643767995038968e-05,0.06824861736398886;
	H << 1.000000000000002,0,0,0,0.6189884034365104,-1.567038190831635e-05,0,-0.1189884044766717,0,0,0,0,0,0,0,0,0,0,0,0,0,0.9999999999999999,0,-0.6189884034365108,0,-0.001661204266203103,0.1189884034448436,-2.187751434606082e-07,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.9999999999954489,1.567038165894966e-05,0.001661204268588177,0,-1.567038293582716e-05,-0.001661204252212605,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1.000097450847848,1.83880491670847e-06,0,-1.000097450847848,-1.838804902640196e-06,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1.838625779404502e-06,0.9999999999983099,0,1.838625719099091e-06,-0.9999999913264482,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-0.01396105997948919,-2.566916426112241e-08,0.9999999999999998,0.01396105997948914,0.0001317223833560791,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.9999025586445076,1.838446583670394e-06,0.01395969959516166,-0.9999025586445076,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.9999999913279877,-0.0001316967141875097,0,-0.9999999999999999,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-0.01395969971621931,0.0001316838814811654,0.9999025499733402,0.01395969971621931,0,0,0,0,0,0,0,0,0,9.192232728802814e-07,-0.4999512749781554,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.4999999956556586,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,6.584193990794745e-05,0.006979849798890586,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1.000000000000024,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.9999999999999195,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.9999999999999999,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.9999999999999999;
	
	// Augmented system
	Eigen::MatrixXd Anu(23,23);
	Eigen::MatrixXd Bnu(23,4);
	Eigen::MatrixXd Hnu(16,23);
	
	Anu << A,                           F, 
	       Eigen::MatrixXd::Zero(3,20), Eigen::MatrixXd::Identity(3,3);
		  
        Bnu << B, 
	       Eigen::MatrixXd::Zero(3,4);
	
	Hnu << H, Eigen::MatrixXd::Zero(16,3);	  

	// Initialization of variables
	Eigen::MatrixXd xhat_minus(23,1);
	//Eigen::MatrixXd xhat_plus(23,1);
	Eigen::MatrixXd Pxx_minus(23,23);
	//Eigen::MatrixXd Pxx_plus(23,23);
	Eigen::MatrixXd K(23,16);
	//Eigen::MatrixXd Pyy_minus(23,23);
	
	int nof_measurements = I.size();
	
	// Turning vector<double> y into MatrixXd object
	Eigen::MatrixXd y_mtx(nof_measurements,1);
	for (int i=0; i<nof_measurements; i++)
	{
		y_mtx(i) = y.at(i);
	}
	
	// Building Hnu[k] and PIeq[k]
	Eigen::MatrixXd Hk(nof_measurements,20);
	Eigen::MatrixXd Hnuk(nof_measurements,23);
	Eigen::MatrixXd PIeqk(nof_measurements,1);
	for (int i=0; i<nof_measurements; i++)
	{
		Hk.row(i) = H.row(I.at(i));
		PIeqk.row(i) = PIeq.row(I.at(i));
	}	
	Hnuk << Hk, Eigen::MatrixXd::Zero(nof_measurements,3);
	
	// Resizing the Kalman gain matrix
	K.resize(23,nof_measurements);
	
	// Building R[k]
	Eigen::MatrixXd Rk(nof_measurements,nof_measurements);
	Rk.setZero();
	for (int i=0; i<nof_measurements; i++)
	{
		Rk(i,i) = R(I.at(i),I.at(i));
	}		
	
	
	// Prediction 
	xhat_minus = Anu*xhat_plus_prev + Bnu*u_prev;
	
	// Covariance (prediction)
	Pxx_minus = Anu*Pxx_plus_prev*Anu.transpose() + Q;
	
	// Kalman gain
	//K = Pxx_minus*Hnu.transpose()*(Hnu*Pxx_minus*Hnu.transpose() + R).inverse();
	K = Pxx_minus*Hnuk.transpose()*(Hnuk*Pxx_minus*Hnuk.transpose() + Rk).inverse();
	
	// Correction
	//xhat_plus = xhat_minus + K*(y_mtx - Hnu*xhat_minus);
	//xhat_plus = xhat_minus + K*(y_mtx - Hnuk*xhat_minus);
	xhat_plus = xhat_minus + K*(y_mtx - (Hnuk*xhat_minus + PIeqk)); // Added the equilibrium outputs
	
	// Covariance (correction)
	//Pxx_plus = (Eigen::MatrixXd::Identity(23,23) - K*Hnu)*Pxx_minus*(Eigen::MatrixXd::Identity(23,23) - K*Hnu).transpose() + K*R*K.transpose();
	Pxx_plus = (Eigen::MatrixXd::Identity(23,23) - K*Hnuk)*Pxx_minus*(Eigen::MatrixXd::Identity(23,23) - K*Hnuk).transpose() + K*Rk*K.transpose();
	
}

};	
