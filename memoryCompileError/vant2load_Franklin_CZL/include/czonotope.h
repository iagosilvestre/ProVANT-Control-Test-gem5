#include <iostream>
#include <Eigen/Eigen>
#include "math.h"
#include "stdio.h"
//#include <random>
//#include "simulator_msgs/Sensor.h"
//#include <ilcplex/ilocplex.h>
#include <limits>

//using namespace Eigen;

/* A first implementation of constrained zonotopes in C++.

   Author: Brenner Santana Rego
   E-mail: brennersr7@ufmg.br   */


#ifndef CZONOTOPE_H
#define CZONOTOPE_H

struct cz {
	Eigen::MatrixXd c;
	Eigen::MatrixXd G;
	Eigen::MatrixXd A;
	Eigen::MatrixXd b;
}; 

struct liftz {
	Eigen::MatrixXd c;
	Eigen::MatrixXd G;
        int dimension;
}; 

class czonotope
{

        public:
        
	czonotope()
	{ 
		
	}
	~czonotope()
	{
		
	}
	
        static cz create(Eigen::MatrixXd c, Eigen::MatrixXd G, Eigen::MatrixXd A, Eigen::MatrixXd b);

        static void print(cz Z);

        static cz limage(cz Z, Eigen::MatrixXd R);

        static cz sum(cz Z, cz W);
         
        static cz intersection(cz Z, cz Y, Eigen::MatrixXd R);

        static liftz lift(cz Z);

        static cz delift(liftz Zlift);

        static void printlift(liftz Z);

        static Eigen::MatrixXd intervalhull(cz Z);

        static bool isempty(cz Z);

        static bool belongsto(cz Z, Eigen::MatrixXd z);
        
        static cz rescaling(cz Z);        
        
//        static cz libczonscale(cz Z);
        static void libczonscale(cz& Z, Eigen::MatrixXd& xiL_Score, Eigen::MatrixXd& xiU_Score);

        static cz creduction(cz Z, int nc_max);

        static cz greduction(cz Z, int ng_max);

        static cz libczonscaledualize(cz Z);

        static liftz libzonreducechisci(liftz Z, double zo);

//      static cz libzonreducesimple(cz Z, int max_g);

        static Eigen::MatrixXd partialsolve(Eigen::MatrixXd A, int NR, int NC);
        
        static cz libczonrmzeroconstr(cz Z);

        static void cplexwrapper(Eigen::MatrixXd f, Eigen::MatrixXd A, Eigen::MatrixXd b, Eigen::MatrixXd Aeq, Eigen::MatrixXd beq, Eigen::MatrixXd lb, Eigen::MatrixXd ub, double& objvalue, Eigen::MatrixXd& xstar, int& exitflag);
        
        static void cplexwrapper2(Eigen::MatrixXd f, Eigen::MatrixXd A, Eigen::MatrixXd b, Eigen::MatrixXd Aeq, Eigen::MatrixXd beq, Eigen::MatrixXd LB, Eigen::MatrixXd UB, double& objvalue, Eigen::MatrixXd& xstar, int& exitflag);
        
        static void cplexwrapper2minmax(Eigen::MatrixXd f, Eigen::MatrixXd A, Eigen::MatrixXd b, Eigen::MatrixXd Aeq, Eigen::MatrixXd beq, Eigen::MatrixXd LB, Eigen::MatrixXd UB, double& objvaluemin, double& objvaluemax, Eigen::MatrixXd& xstarmin, Eigen::MatrixXd& xstarmax, int& exitflag);
        
        static void cplexwrapper2minmax_manyf_fixconst(Eigen::MatrixXd f, Eigen::MatrixXd A, Eigen::MatrixXd b, Eigen::MatrixXd Aeq, Eigen::MatrixXd beq, Eigen::MatrixXd LB, Eigen::MatrixXd UB, Eigen::MatrixXd& objvaluemin, Eigen::MatrixXd& objvaluemax, int& exitflag);                
        
        static void cplexqpwrapper(Eigen::MatrixXd H, Eigen::MatrixXd f, Eigen::MatrixXd A, Eigen::MatrixXd b, Eigen::MatrixXd Aeq, Eigen::MatrixXd beq, Eigen::MatrixXd LB, Eigen::MatrixXd UB, double& objvalue, Eigen::MatrixXd& xstar, int& exitflag);
        
        static void luwrapper(Eigen::MatrixXd A, Eigen::MatrixXd& L, Eigen::MatrixXd& U, Eigen::PermutationMatrix<Eigen::Dynamic,Eigen::Dynamic>& P);
        
        static void free_and_null(char **ptr);

};	

#endif

