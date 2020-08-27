#include <iostream>
#include <Eigen/Eigen>
#include "math.h"

#ifndef FEEDFORWARD_H
#define FEEDFORWARD_H

class feedforward
{
	public:

	feedforward()
	{ 
		
	}
	~feedforward()
	{
		
	}
	
	static Eigen::MatrixXd compute(Eigen::MatrixXd qref, Eigen::MatrixXd qrefdot, Eigen::MatrixXd qrefddot);
	
};

#endif