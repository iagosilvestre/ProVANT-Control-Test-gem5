#include "czonotope.h"

//using namespace Eigen;

/* A first implementation of constrained zonotopes in C++.

   Author: Brenner Santana Rego
   E-mail: brennersr7@ufmg.br   */

//struct cz {
//	Eigen::MatrixXd c;
//	Eigen::MatrixXd G;
//	Eigen::MatrixXd A;
//	Eigen::MatrixXd b;
//}; 

//struct liftz {
//	Eigen::MatrixXd c;
//	Eigen::MatrixXd G;
//        int dimension;
//}; 

//class czonotope
//{
//	public: czonotope()
//	{ 
//		
//	}
//	public: ~czonotope()
//	{
//		
//	}
	
cz czonotope::create(Eigen::MatrixXd c, Eigen::MatrixXd G, Eigen::MatrixXd A, Eigen::MatrixXd b)
{
    /* Creates a structure object with fields c, G, A and b
	   corresponding to a constrained zonotope described by
	   x = c + Gz : norm(z,inf) <= 1, Az = b} */
	
	cz Z;
	Z.c = c;
	Z.G = G;
	Z.A = A;
	Z.b = b;
	
	return Z;
}

// public: static void print(cz Z)
// {
	
	// printf("\n c = ["); std::cout << Z.c; printf("]\n");
	// printf("\n G = ["); std::cout << Z.G; printf("]\n");
	// printf("\n A = ["); std::cout << Z.A; printf("]\n");
	// printf("\n b = ["); std::cout << Z.b; printf("]\n");		
// }

void czonotope::print(cz Z)
{
	
	printf("\n c = \n"); std::cout << Z.c; printf("\n");
	printf("\n G = \n"); std::cout << Z.G; printf("\n");
	printf("\n A = \n"); std::cout << Z.A; printf("\n");
	printf("\n b = \n"); std::cout << Z.b; printf("\n");		
}

cz czonotope::limage(cz Z, Eigen::MatrixXd R)
{
	/* Returns the linear image of a constrained zonotope */
	
	//Eigen::MatrixXd c_new(R.rows(),1);
	//Eigen::MatrixXd G_new(R.rows(), Z.G.cols());	
	
	cz Znew = create(R*Z.c, R*Z.G, Z.A, Z.b);
	return Znew;
		
}

cz czonotope::sum(cz Z, cz W)
{
	/* Returns the Minkowski sum of two constrained zonotopes */	
	
	Eigen::MatrixXd c_new;
	Eigen::MatrixXd G_new;
	Eigen::MatrixXd A_new;
	Eigen::MatrixXd b_new;
	
	if((Z.A.rows()==0)&&(W.A.rows()==0)) // If both Z and W are zonotopes
	{
	        c_new.resize(Z.c.rows(),1);
	        G_new.resize(Z.c.rows(), Z.G.cols() + W.G.cols());
	
	        c_new = Z.c + W.c;
	        G_new << Z.G, W.G;
	        
	}
	else if((Z.A.rows()==0)&&(W.A.rows()!=0)) // If Z is a zonotope and W is a constrained zonotope
	{
        	c_new.resize(Z.c.rows(),1);
	        G_new.resize(Z.c.rows(), Z.G.cols() + W.G.cols());
	        A_new.resize(W.A.rows(), Z.G.cols() + W.A.cols());
	        b_new.resize(W.b.rows(), 1);
	
	        c_new = Z.c + W.c;
	        G_new << Z.G, W.G;
	        A_new << Eigen::MatrixXd::Zero(W.A.rows(), Z.A.cols()),       W.A;
	        b_new << W.b;
	
	}	
	else if((Z.A.rows()!=0)&&(W.A.rows()==0)) // If Z is a constrained zonotope and W is a zonotope
	{
	        c_new.resize(Z.c.rows(),1);
	        G_new.resize(Z.c.rows(), Z.G.cols() + W.G.cols());
	        A_new.resize(Z.A.rows(), Z.A.cols() + W.G.cols());
	        b_new.resize(Z.b.rows(), 1);
	
	        c_new = Z.c + W.c;
	        G_new << Z.G, W.G;
	        A_new << Z.A,       Eigen::MatrixXd::Zero(Z.A.rows(), W.G.cols());
	        b_new << Z.b;	
	
	}	
	else // If Z and W are constrained zonotopes
	{
	      	c_new.resize(Z.c.rows(),1);
	        G_new.resize(Z.c.rows(), Z.G.cols() + W.G.cols());
	        A_new.resize(Z.A.rows() + W.A.rows(), Z.A.cols() + W.A.cols());
	        b_new.resize(Z.b.rows() + W.b.rows(), 1);
	
	        c_new = Z.c + W.c;
	        G_new << Z.G, W.G;
	        A_new << Z.A,       Eigen::MatrixXd::Zero(Z.A.rows(), W.A.cols()),
                     Eigen::MatrixXd::Zero(W.A.rows(), Z.A.cols()),       W.A;
	        b_new << Z.b, W.b;
	        
	}
	
//	Eigen::MatrixXd c_new(Z.c.rows(),1);
//	Eigen::MatrixXd G_new(Z.c.rows(), Z.G.cols() + W.G.cols());
//	Eigen::MatrixXd A_new(Z.A.rows() + W.A.rows(), Z.A.cols() + W.A.cols());
//	Eigen::MatrixXd b_new(Z.b.rows() + W.b.rows(), 1);
//	
//	c_new = Z.c + W.c;
//	G_new << Z.G, W.G;
//	A_new << Z.A,       Eigen::MatrixXd::Zero(Z.A.rows(), W.A.cols()),
//             Eigen::MatrixXd::Zero(W.A.rows(), Z.A.cols()),       W.A;
//	b_new << Z.b, W.b;
	
	cz Znew = create(c_new, G_new, A_new, b_new);
	return Znew;
		
} 
 
cz czonotope::intersection(cz Z, cz Y, Eigen::MatrixXd R)
{
	/* Returns the generalized intersection of two
       constrained zonotopes */
       
	Eigen::MatrixXd c_new;
	Eigen::MatrixXd G_new;
	Eigen::MatrixXd A_new;
	Eigen::MatrixXd b_new;       
       
       	if((Z.A.rows()==0)&&(Y.A.rows()==0)) // If both Z and Y are zonotopes
       	{
	        c_new.resize(Z.c.rows(),1);
	        G_new.resize(Z.c.rows(), Z.G.cols() + Y.G.cols());
	        A_new.resize(R.rows(), Z.G.cols() + Y.G.cols());
	        b_new.resize(R.rows(), 1);
	
	        c_new = Z.c;
	        G_new << Z.G, Eigen::MatrixXd::Zero(Z.G.rows(),Y.G.cols());
	        A_new << R*Z.G, -Y.G; 
	        b_new << Y.c - R*Z.c;       	
		         
       	}
	else if((Z.A.rows()==0)&&(Y.A.rows()!=0)) // If Z is a zonotope and Y is a constrained zonotope     
	{
        	c_new.resize(Z.c.rows(),1);
	        G_new.resize(Z.c.rows(), Z.G.cols() + Y.G.cols());
	        A_new.resize(Y.A.rows() + R.rows(), Z.G.cols() + Y.G.cols());
	        b_new.resize(Y.b.rows() + R.rows(), 1);
	
	        c_new = Z.c;
	        G_new << Z.G, Eigen::MatrixXd::Zero(Z.G.rows(),Y.G.cols());
	        A_new << Eigen::MatrixXd::Zero(Y.A.rows(), Z.G.cols()),                                             Y.A,
                                                                 R*Z.G,                                            -Y.G; 
	        b_new << Y.b,
		         Y.c - R*Z.c;
		         
	}
	else if((Z.A.rows()!=0)&&(Y.A.rows()==0)) // If Z is a constrained zonotope and Y is a zonotope 
	{
        	c_new.resize(Z.c.rows(),1);
	        G_new.resize(Z.c.rows(), Z.G.cols() + Y.G.cols());
	        A_new.resize(Z.A.rows() + R.rows(), Z.G.cols() + Y.G.cols());
	        b_new.resize(Z.b.rows() + R.rows(), 1);
	
	        c_new = Z.c;
	        G_new << Z.G, Eigen::MatrixXd::Zero(Z.G.rows(),Y.G.cols());
	        A_new << Z.A,                                        Eigen::MatrixXd::Zero(Z.A.rows(), Y.G.cols()),
                                                             R*Z.G,                                           -Y.G; 
	        b_new << Z.b,
		         Y.c - R*Z.c;
			         
	}
	else // If Z and W are constrained zonotopes	 	
	{
		c_new.resize(Z.c.rows(),1);
	        G_new.resize(Z.c.rows(), Z.G.cols() + Y.G.cols());
	        A_new.resize(Z.A.rows() + Y.A.rows() + R.rows(), Z.A.cols() + Y.A.cols());
	        b_new.resize(Z.b.rows() + Y.b.rows() + R.rows(), 1);
	
	        c_new = Z.c;
	        G_new << Z.G, Eigen::MatrixXd::Zero(Z.G.rows(),Y.G.cols());
	        A_new << Z.A,                                             Eigen::MatrixXd::Zero(Z.A.rows(), Y.A.cols()),
                     Eigen::MatrixXd::Zero(Y.A.rows(), Z.A.cols()),                                             Y.A,
                                                             R*Z.G,                                            -Y.G; 
	        b_new << Z.b,
                  	 Y.b,
		         Y.c - R*Z.c;
		         
	}
	
//	Eigen::MatrixXd c_new(Z.c.rows(),1);
//	Eigen::MatrixXd G_new(Z.c.rows(), Z.G.cols() + Y.G.cols());
//	Eigen::MatrixXd A_new(Z.A.rows() + Y.A.rows() + R.rows(), Z.A.cols() + Y.A.cols());
//	Eigen::MatrixXd b_new(Z.b.rows() + Y.b.rows() + R.rows(), 1);
//	
//	c_new = Z.c;
//	G_new << Z.G, Eigen::MatrixXd::Zero(Z.G.rows(),Y.G.cols());
//	A_new << Z.A,                                             Eigen::MatrixXd::Zero(Z.A.rows(), Y.A.cols()),
//             Eigen::MatrixXd::Zero(Y.A.rows(), Z.A.cols()),                                             Y.A,
//                                                     R*Z.G,                                            -Y.G; 
//	b_new << Z.b,
//          	 Y.b,
//		 Y.c - R*Z.c;
	
	cz Znew = create(c_new, G_new, A_new, b_new);
	return Znew;
		
}

liftz czonotope::lift(cz Z)
{
	/* Returns the lifted zonotope associated to a constrained
           zonotope */
           
        Eigen::MatrixXd c(Z.c.rows() + Z.b.rows(), 1);
        Eigen::MatrixXd G(Z.G.rows() + Z.A.rows(), Z.G.cols());
        c << Z.c, -Z.b;
        G << Z.G,  Z.A;        
        
        liftz Zlifted;
        Zlifted.c = c;
        Zlifted.G = G;
        Zlifted.dimension = Z.G.rows();
        
        return Zlifted;
           
//      Zlifted.c = [Z.c; -Z.b];
//      Zlifted.G = [Z.G;  Z.A];
//      Zlifted.dimension = size(Z.G,1);
	
		
}

cz czonotope::delift(liftz Zlift)
{
	/* Returns the constrained zonotope associated to a lifted
           zonotope */
           
        Eigen::MatrixXd c(Zlift.dimension, 1);
        Eigen::MatrixXd G(Zlift.dimension, Zlift.G.cols());
        Eigen::MatrixXd A(Zlift.G.rows() - Zlift.dimension, Zlift.G.cols());
        Eigen::MatrixXd b(Zlift.c.rows() - Zlift.dimension, 1);
        c << Zlift.c.block(0, 0, Zlift.dimension, 1);
        G << Zlift.G.block(0, 0, Zlift.dimension, Zlift.G.cols());
        A << Zlift.G.block(Zlift.dimension, 0, Zlift.G.rows() - Zlift.dimension, Zlift.G.cols());
        b << -Zlift.c.block(Zlift.dimension, 0, Zlift.c.rows() - Zlift.dimension, 1);        
        
        cz Z = czonotope::create(c,G,A,b);
        return Z;
        
//      Z.c = Zlifted.c(1:dimension);
//      Z.G = Zlifted.G(1:dimension,:);
//      Z.A = Zlifted.G(dimension+1:end,:);
//      Z.b = -Zlifted.c(dimension+1:end);        	
		
}

void czonotope::printlift(liftz Z)
{
	
	printf("\n c = \n"); std::cout << Z.c; printf("\n");
	printf("\n G = \n"); std::cout << Z.G; printf("\n");
	printf("\n dim = \n"); std::cout << Z.dimension; printf("\n");
}


Eigen::MatrixXd czonotope::intervalhull(cz Z)
{
	/* Returns the smallest box containing a constrained
       zonotope */
	
        int space_dimension = Z.G.rows();
        int nof_generators = Z.G.cols();
        int nof_constraints = Z.A.rows();
        
        
        Eigen::MatrixXd zetalower(space_dimension,1);
        Eigen::MatrixXd zetaupper(space_dimension,1);
        zetalower = Eigen::MatrixXd::Zero(space_dimension,1);
        zetaupper = Eigen::MatrixXd::Zero(space_dimension,1);
        
        
//        Eigen::MatrixXd f(1 + nof_generators, 1);
        Eigen::MatrixXd F(1 + nof_generators, space_dimension);
        
        Eigen::MatrixXd A(2 + 2*nof_generators, 1 + nof_generators);
        Eigen::MatrixXd b(2 + 2*nof_generators, 1);
        
        Eigen::MatrixXd Aeq(nof_constraints, 1 + nof_generators);
        Eigen::MatrixXd beq(nof_constraints, 1);
        
        Eigen::MatrixXd LB(1 + nof_generators,1);
        Eigen::MatrixXd UB(1 + nof_generators,1);
        
//        double objvalue;
//        double objvaluemin;
//        double objvaluemax;                

//        Eigen::MatrixXd xstar(1 + nof_generators,1);
//        Eigen::MatrixXd xstarmin(1 + nof_generators,1);
//        Eigen::MatrixXd xstarmax(1 + nof_generators,1);        
        int exitflag;

        F << Eigen::MatrixXd::Zero(1, space_dimension), Z.G.transpose();
        
        A <<                                        1,                   Eigen::MatrixXd::Zero(1,nof_generators),
                                                   -1,                   Eigen::MatrixXd::Zero(1,nof_generators),
             -Eigen::MatrixXd::Ones(nof_generators,1),  Eigen::MatrixXd::Identity(nof_generators,nof_generators),
             -Eigen::MatrixXd::Ones(nof_generators,1), -Eigen::MatrixXd::Identity(nof_generators,nof_generators);
             
        b << 1, 0, Eigen::MatrixXd::Zero(2*nof_generators,1);
        
        Aeq << Eigen::MatrixXd::Zero(nof_constraints,1), Z.A;
        
        beq << Z.b;

        LB << 0, -Eigen::MatrixXd::Ones(nof_generators,1);
        UB << 1,  Eigen::MatrixXd::Ones(nof_generators,1);   
  
        
//        for(int j=0; j<space_dimension; j++)
//        {
//                
//               f << 0, Z.G.row(j).transpose();
//                
////                A <<                                        1,                   Eigen::MatrixXd::Zero(1,nof_generators),
////                                                           -1,                   Eigen::MatrixXd::Zero(1,nof_generators),
////                     -Eigen::MatrixXd::Ones(nof_generators,1),  Eigen::MatrixXd::Identity(nof_generators,nof_generators),
////                     -Eigen::MatrixXd::Ones(nof_generators,1), -Eigen::MatrixXd::Identity(nof_generators,nof_generators);
////                     
////                b << 1, 0, Eigen::MatrixXd::Zero(2*nof_generators,1);
////                
////                Aeq << Eigen::MatrixXd::Zero(nof_constraints,1), Z.A;
////                
////                beq << Z.b;
//// 
////                LB << 0, -Eigen::MatrixXd::Ones(nof_generators,1);
////                UB << 1,  Eigen::MatrixXd::Ones(nof_generators,1);   
////                
////                // DEBUG
//////                printf("\n"); std::cout << f; printf("\n");         
//////                printf("\n"); std::cout << A; printf("\n");         
//////                printf("\n"); std::cout << b; printf("\n");         
//////                printf("\n"); std::cout << Aeq; printf("\n");         
//////                printf("\n"); std::cout << beq; printf("\n");         
//////                printf("\n"); std::cout << LB; printf("\n");         
//////                printf("\n"); std::cout << UB; printf("\n");         
//                
////                cplexwrapper2(f, A, b, Aeq, beq, LB, UB, objvalue, xstar, exitflag);     zetalower(j,0) =  objvalue;
////                cplexwrapper2(-f, A, b, Aeq, beq, LB, UB, objvalue, xstar, exitflag);    zetaupper(j,0) = -objvalue;
//                        
//                cplexwrapper2minmax(f, A, b, Aeq, beq, LB, UB, objvaluemin, objvaluemax, xstarmin, xstarmax, exitflag);
//                zetalower(j,0) =  objvaluemin;
//                zetaupper(j,0) =  objvaluemax;              
//                
//        }
        
        cplexwrapper2minmax_manyf_fixconst(F, A, b, Aeq, beq, LB, UB, zetalower, zetaupper, exitflag);  
        
        zetalower = zetalower + Z.c;
        zetaupper = zetaupper + Z.c;
        
        Eigen::MatrixXd hull(space_dimension,2);
        hull << zetalower, zetaupper;
        return hull;
        
//    for j=1:space_dimension

//%        f = [0; Z.G(j,:).'];
//        %f(j+1) = 1;
//%         A = [                      1,  zeros(1,nof_generators);
//%              -ones(nof_generators,1),      eye(nof_generators);
//%              -ones(nof_generators,1),     -eye(nof_generators)];
//%         b = [1; zeros(2*nof_generators,1)];
//%         Aeq = [zeros(nof_constraints,1), Z.A];
//%         beq = Z.b;

//        f = [0; Z.G(j,:).'];
//        A = [                      1,  zeros(1,nof_generators);
//                                  -1,  zeros(1,nof_generators);
//             -ones(nof_generators,1),      eye(nof_generators);
//             -ones(nof_generators,1),     -eye(nof_generators)];
//        b = [1; 0; zeros(2*nof_generators,1)];
//        Aeq = [zeros(nof_constraints,1), Z.A];
//        beq = Z.b;
//        
//        LB = [0; -ones(nof_generators,1)];
//        UB = [1;  ones(nof_generators,1)];

//%         [~,zmin] = linprog(f,A,b,Aeq,beq,[],[],[],OPTIONS);  zetalower(j) =  zmin;
//%         [~,zmax] = linprog(-f,A,b,Aeq,beq,[],[],[],OPTIONS); zetaupper(j) = -zmax;

//        % Linprog
//%         [~,zmin] = linprog(f,A,b,Aeq,beq,LB,UB,[],OPTIONS);  zetalower(j) =  zmin;
//%         [~,zmax] = linprog(-f,A,b,Aeq,beq,LB,UB,[],OPTIONS); zetaupper(j) = -zmax;
//        
//        % Cplex
//        [~,zmin] = cplexlp(f,A,b,Aeq,beq,LB,UB,[],OPTIONS);  zetalower(j) =  zmin;
//        [~,zmax] = cplexlp(-f,A,b,Aeq,beq,LB,UB,[],OPTIONS); zetaupper(j) = -zmax;        
//        
//        
//    end

		
}

bool czonotope::isempty(cz Z)
{
	/* Verifies if a constrained zonotope is an empty set */
		
        int nof_generators = Z.G.cols();
        int nof_constraints = Z.A.rows();
        
        Eigen::MatrixXd f(1 + nof_generators, 1);
        
        Eigen::MatrixXd A(1 + 2*nof_generators, 1 + nof_generators);
        Eigen::MatrixXd b(1 + 2*nof_generators, 1);
        
        Eigen::MatrixXd Aeq(nof_constraints, 1 + nof_generators);
        Eigen::MatrixXd beq(nof_constraints, 1);
        
        Eigen::MatrixXd LB;
        Eigen::MatrixXd UB;
        
        f << 1, Eigen::MatrixXd::Zero(nof_generators,1);        
        
        A <<                                       -1,                   Eigen::MatrixXd::Zero(1,nof_generators),
             -Eigen::MatrixXd::Ones(nof_generators,1),  Eigen::MatrixXd::Identity(nof_generators,nof_generators),
             -Eigen::MatrixXd::Ones(nof_generators,1), -Eigen::MatrixXd::Identity(nof_generators,nof_generators);
             
        b << 0, Eigen::MatrixXd::Zero(2*nof_generators,1);
        
        Aeq << Eigen::MatrixXd::Zero(nof_constraints,1), Z.A;
        
        beq << Z.b;        
        
        
        double objvalue;
        Eigen::MatrixXd xstar(1 + nof_generators,1);
        int exitflag;        
                
        cplexwrapper2(f, A, b, Aeq, beq, LB, UB, objvalue, xstar, exitflag);
        
        
//        printf("Optimal value of objective function: \n"); std::cout << objvalue; printf("\n");
//        printf("First entry of xstar is: \n"); std::cout << xstar(0,0); printf("\n");
                
        if(exitflag==0)
        {
                if(objvalue <= 1)
                {
                        return false; // Not empty
                }
                else
                {
                        return true; // Empty
                }                
        }
        else
        {
                return true; // Empty
        }
        
//        f = [1; zeros(nof_generators,1)];
//        A = [                 -1,  zeros(1,nof_generators);
//             -ones(nof_generators,1),  eye(nof_generators);
//             -ones(nof_generators,1), -eye(nof_generators)];
//        b = [0; zeros(2*nof_generators,1)];
//        Aeq = [zeros(nof_constraints,1), Z.A];
//        beq = Z.b;        
		
}

bool czonotope::belongsto(cz Z, Eigen::MatrixXd z)
{
	/* Verifies if a point belongs to a constrained zonotope */
	
	int space_dimension = Z.G.rows();
        int nof_generators = Z.G.cols();
        int nof_constraints = Z.A.rows();
        
        Eigen::MatrixXd f(1 + nof_generators, 1);
        
        Eigen::MatrixXd A(2*nof_generators, 1 + nof_generators);
        Eigen::MatrixXd b(2*nof_generators, 1);
        
        Eigen::MatrixXd Aeq(space_dimension + nof_constraints, 1 + nof_generators);
        Eigen::MatrixXd beq(space_dimension + nof_constraints, 1);
        
        Eigen::MatrixXd LB;
        Eigen::MatrixXd UB;        
        
        f << 1, Eigen::MatrixXd::Zero(nof_generators,1);        
        
        A << -Eigen::MatrixXd::Ones(nof_generators,1),  Eigen::MatrixXd::Identity(nof_generators,nof_generators),
             -Eigen::MatrixXd::Ones(nof_generators,1), -Eigen::MatrixXd::Identity(nof_generators,nof_generators);
             
        b << Eigen::MatrixXd::Zero(2*nof_generators,1);
        
        Aeq << Eigen::MatrixXd::Zero(space_dimension,1), Z.G,
               Eigen::MatrixXd::Zero(nof_constraints,1), Z.A;
        
        beq << z - Z.c,
                   Z.b;        
        
//        space_dimension = size(Z.G,1);
//        nof_generators = size(Z.G,2);
//        nof_constraints = size(Z.A,1);

//        f = [1; zeros(nof_generators,1)];
//        A = [-ones(nof_generators,1),  eye(nof_generators);
//             -ones(nof_generators,1), -eye(nof_generators)];
//        b = zeros(2*nof_generators,1);
//        Aeq = [zeros(space_dimension,1), Z.G;
//               zeros(nof_constraints,1), Z.A];
//        beq = [z - Z.c;
//                   Z.b];

//        %[x,~,exitflag] = linprog(f,A,b,Aeq,beq,[],[],[],OPTIONS);
//        [x,~,exitflag] = cplexlp(f,A,b,Aeq,beq,[],[],[],OPTIONS);

//        if(exitflag==1) % Success
//            norm_csi = x(1);
//            if norm_csi <= 1
//                result = 1;
//            else
//                result = 0;
//            end
//        else % Not success (treated as somehow unfeasible and the point z not belonging to the contrained zonotope Z)
//            %disp(strcat(['Exit flag ',num2str(exitflag),' in CZONOTOPE_BELONGSTO']));
//            result = 0;
//        end        
		        
        
        
        double objvalue;
        Eigen::MatrixXd xstar(1 + nof_generators,1);
        int exitflag;        
                
        cplexwrapper2(f, A, b, Aeq, beq, LB, UB, objvalue, xstar, exitflag);
        
        
//        printf("Optimal value of objective function: \n"); std::cout << objvalue; printf("\n");

        if(exitflag==0)
        {
                if(objvalue <= 1)
                {
                        return true; // Belongs to
                }
                else
                {
                        return false; // Does not belong to
                }                
        }
        else
        {
                return false; // Does not belong to (infeasible)
        }	
        

}

cz czonotope::rescaling(cz Z)
{
        /* Performs rescaling over a constrained zonotope, based
           on linear programming */

        int nof_generators = Z.G.cols();
        int nof_constraints = Z.A.rows();
        
        Eigen::MatrixXd csilower(nof_generators,1);        
        Eigen::MatrixXd csiupper(nof_generators,1);
        csilower = -Eigen::MatrixXd::Ones(nof_generators,1);
        csiupper =  Eigen::MatrixXd::Ones(nof_generators,1);        
        
        
        
        Eigen::MatrixXd f(1 + nof_generators, 1);
        
        Eigen::MatrixXd A(1 + 2*nof_generators, 1 + nof_generators);
        Eigen::MatrixXd b(1 + 2*nof_generators, 1);
        
        Eigen::MatrixXd Aeq(nof_constraints, 1 + nof_generators);
        Eigen::MatrixXd beq(nof_constraints, 1);
        
        Eigen::MatrixXd LB;
        Eigen::MatrixXd UB;           
        
        double objvalue;
        Eigen::MatrixXd xstar(1 + nof_generators,1);
        int exitflag;
        
        for(int j=0; j<nof_generators; j++)
        {    
                f << Eigen::MatrixXd::Zero(1 + nof_generators,1);
                f(j+1,0) = 1;

                A <<                                        1,                   Eigen::MatrixXd::Zero(1,nof_generators),
                     -Eigen::MatrixXd::Ones(nof_generators,1),  Eigen::MatrixXd::Identity(nof_generators,nof_generators),
                     -Eigen::MatrixXd::Ones(nof_generators,1), -Eigen::MatrixXd::Identity(nof_generators,nof_generators);
                     
                b << 1, Eigen::MatrixXd::Zero(2*nof_generators,1);
                
                Aeq << Eigen::MatrixXd::Zero(nof_constraints,1), Z.A;
                
                beq << Z.b;
                
                cplexwrapper2(f, A, b, Aeq, beq, LB, UB, objvalue, xstar, exitflag);     csilower(j) = xstar(j+1);
                cplexwrapper2(-f, A, b, Aeq, beq, LB, UB, objvalue, xstar, exitflag);    csiupper(j) = xstar(j+1);                
        
                
                
        }    
        
        Eigen::MatrixXd csimid(nof_generators,1);
        Eigen::MatrixXd csirad(nof_generators,1);        
        
        csimid = (0.5)*(csilower+csiupper);
        csirad = (0.5)*(csiupper-csilower);
        
        Eigen::MatrixXd c_new;
        Eigen::MatrixXd G_new;       
        Eigen::MatrixXd A_new;
        Eigen::MatrixXd b_new;                
        
        c_new = Z.c + Z.G*csimid;
        G_new = Z.G*csirad.asDiagonal();        
        if(Z.A.rows()!=0) // If Z is a constrained zonotope
        {
                A_new = Z.A*csirad.asDiagonal();
                b_new = Z.b - Z.A*csimid;                
        }

              
        cz Znew = czonotope::create(c_new,G_new,A_new,b_new);
        return Znew;
                   
//        nof_generators = size(Z.G,2);
//        nof_constraints = size(Z.A,1);

//        % Cplex options
//        OPTIONS = cplexoptimset; % Revision 28-11-2017
//        OPTIONS.Display = 'off'; % Revision 28-11-2017

//        csilower = -ones(nof_generators,1);
//        csiupper = ones(nof_generators,1);

//        for j=1:nof_generators

//        f = zeros(nof_generators+1,1);
//        f(j+1) = 1;
//        A = [                      1,  zeros(1,nof_generators);
//             -ones(nof_generators,1),      eye(nof_generators);
//             -ones(nof_generators,1),     -eye(nof_generators)];
//        b = [1; zeros(2*nof_generators,1)];
//        Aeq = [zeros(nof_constraints,1), Z.A];
//        beq = Z.b;

//        %x = linprog(f,A,b,Aeq,beq,[],[],[],OPTIONS);  csilower(j) =  x(j+1);
//        %x = linprog(-f,A,b,Aeq,beq,[],[],[],OPTIONS); csiupper(j) =  x(j+1);

//        x = cplexlp(f,A,b,Aeq,beq,[],[],[],OPTIONS);  csilower(j) =  x(j+1); % Revision 28-11-2017
//        x = cplexlp(-f,A,b,Aeq,beq,[],[],[],OPTIONS); csiupper(j) =  x(j+1); % Revision 28-11-2017


//        end

//        csimid = (csilower+csiupper)/2;
//        csirad = (csiupper-csilower)/2;

//        Znew.c =  Z.c + Z.G*csimid;
//        Znew.G = Z.G*diag(csirad);

//        % Revision 01-12-2017
//        if(isempty(Z.A)) % If Z is a zonotope

//        Znew.A = [];
//        Znew.b = [];

//        else % If Z is a constrained zonotope

//        Znew.A = Z.A*diag(csirad);
//        Znew.b = Z.b - Z.A*csimid;

//        end        

}

//cz czonotope::libczonscale(cz Z)
void czonotope::libczonscale(cz& Z, Eigen::MatrixXd& xiL_Score, Eigen::MatrixXd& xiU_Score)
{
        int n = Z.G.rows();
        int ng = Z.G.cols();
        int nc = Z.A.rows();
       
        
        if(nc==0) // If Z is a zonotope
        {
                //return Z;
                return;
        }
        
        //cz Znew;
        //Eigen::MatrixXd c_new;
        //Eigen::MatrixXd G_new;       
        //Eigen::MatrixXd A_new;
        //Eigen::MatrixXd b_new;
       
        
        double Inf = std::numeric_limits<double>::infinity();
        double Epsilon = std::numeric_limits<double>::epsilon();
        
        double Tol = std::max(nc,ng)*Epsilon*Z.A.rowwise().lpNorm<1>().maxCoeff();
        
                    
        
        //Eigen::MatrixXd xiL_Score(1,ng);
        //Eigen::MatrixXd xiU_Score(1,ng);
        xiL_Score = Eigen::MatrixXd::Ones(1,ng)*Inf;
        xiU_Score = Eigen::MatrixXd::Ones(1,ng)*Inf;
               
        
        
        Eigen::MatrixXd aT;
        double b;
        double aT_norm;
        double aT_normM;
        double xiL_Update;
        double xiU_Update;  
        double xi_L;
        double xi_U;
        double xi_r;
        double xi_m;                
        
        
        for(int k=1;k<3;k++)
        {
                for(int row=0; row<nc; row++)
                {
                       
                       aT = Z.A.row(row);
                       b = Z.b(row,0); //b = Z.b.row(row);
                       aT_norm = aT.lpNorm<1>();
                                                        
                       
                       for(int col=0; col<ng; col++)
                       {
                                
                                if(std::abs(aT(0,col)) > Tol) //if(std::abs(aT.col(col)) > Tol)
                                {
                                
                                        //aT_normM = aT_norm - std::abs(aT(0,col)); //aT_normM = aT_norm - std::abs(aT.col(col));
                                        //xiL_Update = (b/aT.col(col)) - std::abs(aT_normM/aT.col(col));
                                        //xiU_Update = (b/aT.col(col)) + std::abs(aT_normM/aT.col(col));                                
                                
                                        aT_normM = aT_norm - std::abs(aT(0,col)); //aT_normM = aT_norm - std::abs(aT.col(col));
                                        xiL_Update = (b/aT(0,col)) - std::abs(aT_normM/aT(0,col));
                                        xiU_Update = (b/aT(0,col)) + std::abs(aT_normM/aT(0,col));                                        
                                        
                                        
                                        xi_L = std::max(-1.0, xiL_Update);
                                        xi_U = std::min( 1.0, xiU_Update);
                                        
                                        xi_r = 0.5*(xi_U - xi_L);
                                        xi_m = 0.5*(xi_U + xi_L);
                                        
                                        
                                        if(xiL_Update >= -1.0)
                                        {
                                                //xiL_Score.col(col) = 0;
                                                xiL_Score(0,col) = 0.0;
                                        }
                                        else
                                        {
                                                if(xi_r>Tol)
                                                {
                                                        //xiL_Score.col(col) = std::min(xiL_Score.col(col), (std::abs(xiL_Update)-1)/xi_r);
                                                        xiL_Score(0,col) = std::min(xiL_Score(0,col), (std::abs(xiL_Update)-1)/xi_r);
                                                }
                                        }
                                        
                                        
                                        if(xiU_Update <= 1.0)
                                        {
                                                //xiU_Score.col(col) = 0;
                                                xiU_Score(0,col) = 0.0;
                                        }
                                        else
                                        {
                                                if(xi_r>Tol)
                                                {
                                                        //xiU_Score.col(col) = std::min(xiU_Score.col(col), (std::abs(xiU_Update)-1)/xi_r);
                                                        xiU_Score(0,col) = std::min(xiU_Score(0,col), (std::abs(xiU_Update)-1)/xi_r);
                                                }
                                        }
                                        
                                        
                                        if(std::abs(xi_r-1) > Tol)
                                        {
                                                if(xi_r < -Tol) // Empty set
                                                {
                                                        printf("Empty set generated in czonotope::libczonscale");
                                                        Z.c.resize(0,0);
                                                        Z.G.resize(0,0);
                                                        Z.A.resize(0,0);
                                                        Z.b.resize(0,0);                                                                                                                                                                        
                                                        //return Z;
                                                        return;
                                                }
                                                else
                                                {
                                                        xi_r = std::max(0.0,xi_r);
                                                }
                                                
                                                Z.c = Z.c + Z.G.col(col)*xi_m;
                                                Z.b = Z.b - Z.A.col(col)*xi_m;
                                                Z.G.col(col) = Z.G.col(col)*xi_r;
                                                Z.A.col(col) = Z.A.col(col)*xi_r;
                                                
                                        }
                                                                                
                                
                                }
                                
                       }
                                            
                       
                       // Continue here
                       
                       Eigen::MatrixXd::Index maxrow, maxcol;                       
                       double Max = Z.A.cwiseAbs().row(row).maxCoeff(&maxrow, &maxcol);
                       if(Max > Tol)
                       {
                                Z.b.row(row) = Z.b.row(row)/Z.A(row,maxcol);
                                Z.A.row(row) = Z.A.row(row)/Z.A(row,maxcol);                                
                       }
                        
                }
        }
        
        //return Z;        
        return;
}

cz czonotope::creduction(cz Z, int nc_max)
{
	/* Performs constraint reduction */
	
	// Wrapper to iterated use of czonotope::libczonscaledualize
	
	int nc_diff = Z.A.rows() - nc_max;
	
	if(nc_diff <= 0)
	{
	        return Z;
	}
	
	for(int i=0; i<nc_diff; i++)
	{
	        Z = czonotope::libczonscaledualize(Z);
	}
	
	return Z;
	
	
		
}

cz czonotope::greduction(cz Z, int ng_max)
{
	/* Performs generator reduction */
	
	// Wrapper to libzonreducechisci for generator reduction of cz
	
	
        int ng_diff = Z.G.cols() - ng_max;
        
        if(ng_diff <= 0)
        {
                return Z;
        }
	
        double zo = ((double) ng_max)/((double) (Z.G.rows() + Z.A.rows()));
        
        Z = czonotope::delift(czonotope::libzonreducechisci(czonotope::lift(Z), zo));	
        
        return Z;
		
}

cz czonotope::libczonscaledualize(cz Z)
{
        /* Implementation of libCZon_ScaleDualize3.m in C++ */
        
        int flag = 0;
        int xi_elim;
        
        double Inf = std::numeric_limits<double>::infinity();        
        
        
        // Get sizes
        int n = Z.G.rows();
        int ng = Z.G.cols();        
        int nc = Z.A.rows();
        //std::cout << n << std::endl;
        //std::cout << ng << std::endl;
        //std::cout << nc << std::endl;                
        
        // Partial solve and scaling
        Eigen::MatrixXd AG = partialsolve((Eigen::MatrixXd(nc+n,ng+1) << Z.A, -Z.b, Z.G, Z.c).finished(), nc, ng);
        Z.c =  AG.block(nc,ng,n,1);
        Z.G =  AG.block(nc,0,n,ng);        
        Z.A =  AG.block(0,0,nc,ng);
        Z.b = -AG.block(0,ng,nc,1);
        //std::cout << AG << std::endl;
        
        
        // Scale
	Eigen::MatrixXd xiL_Score(1,ng);
	Eigen::MatrixXd xiU_Score(1,ng);
	czonotope::libczonscale(Z, xiL_Score, xiU_Score);
        //Z = czonotope::libczonscale(Z);
        //czonotope::print(Z);        
        
        
        // Remove trivial constraints
        int nc0 = nc;
        Z = czonotope::libczonrmzeroconstr(Z);
        nc = Z.A.rows();
        //std::cout << "nc " << nc << std::endl;        
        
        // Number of constraints left do dualize
        int N_Dual = 1 - (nc0-nc);
        N_Dual = std::min(N_Dual,nc);
        if(N_Dual <= 0)
        {
                return Z;
        }
        //std::cout << "N_Dual " << N_Dual << std::endl;        
        
        
        
        // Find best xi to eliminate
        
        // CONTINUE HERE
        
//      //Index array of max(xiL_Score, xiU_Score)double Max = A.block(row,row,NR-row,NC-row).cwiseAbs().rowwise().maxCoeff();
//       xi_elim.resize(NR-row,1);
//       for(int j=0; j<xi_elim.rows(); j++)
//       {
//                A.block(row,row,NR-row,NC-row).cwiseAbs().row(j).maxCoeff(&maxcol);
//               xi_elim(j) = maxcol;
//       }
        
        Eigen::MatrixXd xi_FScore = (Eigen::MatrixXd(2,ng) << xiL_Score, xiU_Score).finished().colwise().maxCoeff();
        Eigen::MatrixXd::Index minxifscore;
        
        //std::cout << "xi_FScore " << xi_FScore << std::endl;
        
        double FScore = xi_FScore.row(0).minCoeff(&minxifscore);
        int F_xi_elim = minxifscore;
        
        //std::cout << "FScore " << FScore << std::endl;        
        //std::cout << "F_xi_elim " << F_xi_elim << std::endl;        
        
        if(FScore < 1.0e-3)
        {
                xi_elim = F_xi_elim;
        }
        else
        {
                
                // Need to compute G_Scores
                Eigen::MatrixXd G = Z.G;
                Eigen::MatrixXd A = Z.A;
                nc = A.rows();
                Eigen::MatrixXd ei = Eigen::MatrixXd::Zero(ng+nc, 1);
                
                Eigen::MatrixXd MMsub(ng + nc, ng + nc);
                Eigen::MatrixXd Mbsub(ng + nc, 1);
                
                MMsub << (G.transpose()*G + Eigen::MatrixXd::Identity(ng,ng)), A.transpose(),
                                                             A, Eigen::MatrixXd::Zero(nc,nc);
                                                             
                Mbsub << Eigen::MatrixXd::Zero(ng + nc, 1);
                
                //std::cout << "nc " << nc << std::endl;                 
                //std::cout << "ei " << ei << std::endl;                 
                //std::cout << "MMsub " << MMsub << std::endl;                                                 
                //std::cout << "Mbsub " << Mbsub << std::endl;                                                                 
                
                
                // LU decomposition (figure out)
                Eigen::MatrixXd L(ng+nc,ng+nc);
                Eigen::MatrixXd U(ng+nc,ng+nc);                
                Eigen::PermutationMatrix<Eigen::Dynamic,Eigen::Dynamic> P(ng+nc);
                luwrapper(MMsub, L, U, P);
                
                //std::cout << "L " << L << std::endl;                                                 
                //std::cout << "U " << U << std::endl;                                                 
                //std::cout << "P " << P.indices() << std::endl;                                                                                 
                
                Eigen::MatrixXd Mx = Eigen::MatrixXd::Zero(ng,1);
                
                
                Eigen::MatrixXd xi_GScore(1,ng);
                for(int i=0; i<ng; i++)
                {
                        if(xi_FScore(i) == Inf)
                        {
                                xi_GScore(i) = Inf;
                        }
                        else
                        {
                                ei(i) = 1.0;
                                
                                Eigen::MatrixXd uhat = L.partialPivLu().solve(P*ei);
                                Eigen::MatrixXd Ihat = U.partialPivLu().solve(uhat);                               

                                
                                Mx = Ihat.block(0,0,ng,1)*(xi_FScore(i)/Ihat(i,0));
                                
                                xi_GScore(i) = (G*Mx).norm();
                                
                                ei(i) = 0;
                                
                                //if(i==0)
                                //{
                                        //std::cout << "P*ei" << P*ei << std::endl;
                                
                                        //std::cout << "uhat " << uhat << std::endl; 
                                        //std::cout << "Ihat " << Ihat << std::endl; 
                                        
                                        //std::cout << "xi_GScore(i) " << xi_GScore(i) << std::endl;                                                
                                        
                                //}                                
                                
                                
                                //A.colPivHouseholderQr().solve(b)
                        }
                }
                
                //std::cout << "xi_GScore " << xi_GScore << std::endl;                 
                
                Eigen::MatrixXd::Index minxigscore;
                xi_GScore.row(0).minCoeff(&minxigscore);
                xi_elim = minxigscore;
                
                //std::cout << "xi_elim " << xi_elim << std::endl;                   
                
        }
        
        
        // Eliminate
        
        double max_val;
        int pivot_row;
        
        Eigen::MatrixXd::Index max_val_index;
        
        if(xi_elim > nc-1)
        {
                max_val = Z.A.cwiseAbs().col(xi_elim).maxCoeff(&max_val_index);
                pivot_row = max_val_index;
                if(max_val == 0)
                {
                        printf("Zero pivot in czonotope::libczonscaledualize");
                        return Z;   
                }
                
                //std::cout << "Z.A.cwiseAbs().col(xi_elim)" << Z.A.cwiseAbs().col(xi_elim) << std::endl;                
                //std::cout << "max_val " << max_val << std::endl; 
                //std::cout << "pivot_row " << pivot_row << std::endl; 
                
                Z.b(pivot_row,0) = Z.b(pivot_row,0)/Z.A(pivot_row,xi_elim);
                Z.A.row(pivot_row) = Z.A.row(pivot_row)/Z.A(pivot_row,xi_elim);        
                
                double lambda;
                for(int row=0; row<pivot_row; row++)
                {
                        // CONTINUE HERE
                        lambda = Z.A(row,xi_elim);
                        Z.b(row,0) = Z.b(row,0) - lambda*Z.b(pivot_row,0);
                        Z.A.row(row) = Z.A.row(row) - lambda*Z.A.row(pivot_row);                
                }
                
                for(int row=pivot_row+1; row<nc; row++)
                {        
                        lambda = Z.A(row,xi_elim);
                        Z.b(row,0) = Z.b(row,0) - lambda*Z.b(pivot_row,0);
                        Z.A.row(row) = Z.A.row(row) - lambda*Z.A.row(pivot_row);                            
                }
                
                for(int row=0; row<n; row++)
                {        
                        lambda = Z.G(row,xi_elim);
                        Z.c(row,0) = Z.c(row,0) + lambda*Z.b(pivot_row,0);
                        Z.G.row(row) = Z.G.row(row) - lambda*Z.A.row(pivot_row);                            
                }    
        }
        else
        {
                pivot_row = xi_elim;
        }
        
        
        // Remove constraint and generator
        
        Z.G.block(0,xi_elim,n,ng-xi_elim-1) = Z.G.rightCols(ng-xi_elim-1);
        Z.G.conservativeResize(n,ng-1);
        
        Z.A.block(0,xi_elim,nc,ng-xi_elim-1) = Z.A.rightCols(ng-xi_elim-1);
        Z.A.conservativeResize(nc,ng-1);
        
        Z.A.block(pivot_row,0,nc-pivot_row-1,ng-1) = Z.A.bottomRows(nc-pivot_row-1);
        Z.A.conservativeResize(nc-1,ng-1);
        
        Z.b.block(pivot_row,0,nc-pivot_row-1,1) = Z.b.bottomRows(nc-pivot_row-1);                           
        Z.b.conservativeResize(nc-1,1);
        
        
        return Z;
}


liftz czonotope::libzonreducechisci(liftz Z, double zo)
{
        /* Implementation of libZon_ReduceOrderChisci.m in C++ */
        
        int flag = 0;
        int n = Z.G.rows();
        int ng = Z.G.cols();
//      int nc = Z.A.rows();
        
//        if(nc > 0) // If Z is not a zonotope
//        {
//                printf("Constrained zonotope detected in czonotope::libzonreducechisci");
//                return Z;
//        }
        
        if (n <= 0 || ng <= 0)
        {
                printf("Error in czonotope::libzonreducechisci");
                flag = -1;
                return Z;
        }
        
        if(zo < 1)
        {
                printf("Desired order less than one in czonotope::libzonreducechisci");
                flag = -1;
                return Z;
        }
        
        
        // Number of generators to eliminate
        int N_Elim = round(ng-zo*n); //std::cout << "N_Elim " << N_Elim << std::endl;
        if(N_Elim <= 0)
        {
                return Z;
        }
        

        // Since the original PartialSolve code does not attibute flag = -4 in any part, the following code is omitted
//        %Find an invertible subset of generators
//        [Dual,flag,~,pivot_record]=PartialSolve([Z{2} eye(n)],n,ng);
//        if flag==-4
//        %There is no invertible set; resort to simple reduction
//        disp(['Warning: ',FuncID,' resorting to libZon_ReduceOrderSimple'])
//        [Z,ng,flag] = libZon_ReduceOrderSimple( Z , zo );
//        return;
//        else
//        if (flag); return; end;
//        end
        Eigen::MatrixXd Dual = partialsolve((Eigen::MatrixXd(n,ng+n) << Z.G, Eigen::MatrixXd::Identity(n,n)).finished(), n, ng);


        //At this stage, n generators are in ParTope, while the rest are in
        //Ordered_Gen. The idea is to remove one generator at a time form
        //Ordered_Gen, add it to ParTope to generate a matrix with n+1
        //generators called ParTopePlus, and apply Chisci's method for optimally
        //enclosing ParTopePlus with a parallelotope. This updates ParTope, and
        //the process is repeated until N_Elim generators have been eliminated.
        
        //The loop below eliminates one generator from Ordered_Gen in each pass
        //by adding it to ParTope and reducing back to a parallelotope.        
        for(int i=0; i<N_Elim; i++)
        {
                
                //Which generator in Ordered_Gen should be added to ParTope for
                //reduction? Can argue that t is a good generator if column of
                //ReductionDesirabilityMeasure is either very small, very large, or
                //nearly a scaled unit vector. Simple hueristic below considers only
                //the first two cases.
                //[ReductionDesirabilityMeasure, CondInv] = linsolve(ParTope,Other_Gen);
                Eigen::MatrixXd R = Dual.block(0,n,n,ng-n).cwiseAbs();//  abs(Dual(:,n+1:ng));
                Eigen::MatrixXd RDM = (Eigen::MatrixXd::Ones(R.rows(), R.cols()) + R).colwise().prod() - Eigen::MatrixXd::Ones(1,R.cols()) - R.colwise().sum();
                
                // Find max and min element of ReductionDesirabilityMeasure
                Eigen::MatrixXd::Index min_col_index;
                RDM.row(0).minCoeff(&min_col_index);
                int col_index = min_col_index;
                
                // Remove generator
                Dual.block(0,n+col_index,n,ng+n-(n+col_index)-1) = Dual.rightCols(ng+n-(n+col_index)-1);
                Dual.conservativeResize(n,ng+n-1);               
                ng--;
                
                // Reduce n+1 generators in ParTopePlus = [ParTope tnp1] to n using
                // Chisci's parallelotope rule
                Dual.block(0,n,Dual.rows(),ng) = (Eigen::MatrixXd::Ones(n,1) + R.col(col_index)).cwiseInverse().asDiagonal()*Dual.block(0,n,n,ng);
                               
        }
        
        // Reform zonotope generator matrix
        Z.G = Dual.block(0,ng,n,n).inverse()*Dual.block(0,0,n,ng);
        
        return Z;        
        
                
}

Eigen::MatrixXd czonotope::partialsolve(Eigen::MatrixXd A, int NR, int NC)
{
        /* Implementation of PartialSolve.m in C++ */

        int flag = 0;
        int nr = A.rows();
        int nc = A.cols();
        
        Eigen::MatrixXd col_pivots(1,nc);
        Eigen::MatrixXd row_pivots(1,nr);
        col_pivots = Eigen::VectorXd::LinSpaced(nc,0,nc-1).transpose();
        row_pivots = Eigen::VectorXd::LinSpaced(nr,0,nr-1).transpose();        
        //col_pivots = Eigen::VectorXd::LinSpaced(nc,0,nc-1);
        //row_pivots = Eigen::VectorXd::LinSpaced(nr,0,nr-1);                
        
        
        // Check inputs
        if (nr==0 || nr>nc || NC>nc || NR>nr)
        {
                printf("Error in czonotope::partialsolve");
                return A;                
        }
        
        
        
        double Inf = std::numeric_limits<double>::infinity();
        double Epsilon = std::numeric_limits<double>::epsilon();
        
        double Tol = std::max(nr,nc)*Epsilon*A.rowwise().lpNorm<1>().maxCoeff();        
        
//        double 

        Eigen::MatrixXi xi_elim;
        Eigen::MatrixXd::Index maxcol;  
        
        Eigen::MatrixXd A_Scaled;
        Eigen::MatrixXd A_Norms;
        Eigen::MatrixXd A_Norms_ordered;
        
        Eigen::MatrixXd::Index minrow; 
        
        int pivot_row;
        int pivot_col;
        
        Eigen::PermutationMatrix<Eigen::Dynamic,Eigen::Dynamic> permuterow(nr);
        Eigen::PermutationMatrix<Eigen::Dynamic,Eigen::Dynamic> permutecol(nc);
        
        
        for(int row=0; row<NR; row++)
        {
               // Find the best pivot
               
               //Index array of double Max = A.block(row,row,NR-row,NC-row).cwiseAbs().rowwise().maxCoeff();
               xi_elim.resize(NR-row,1);
               for(int j=0; j<xi_elim.rows(); j++)
               {
                        A.block(row,row,NR-row,NC-row).cwiseAbs().row(j).maxCoeff(&maxcol);
                        xi_elim(j) = maxcol;
               }
               
               //xi_elim = row - 1 + xi_elim;
               //xi_elim = (row - 1)*Eigen::MatrixXi::Ones(xi_elim.rows(),1) + xi_elim;
               xi_elim = (row)*Eigen::MatrixXi::Ones(xi_elim.rows(),1) + xi_elim;
               A_Scaled = A;
               //A_Scaled << A;
               //A_Norms = Eigen::MatrixXd::Ones(NR+1-row, 1);
               A_Norms.resize(NR-row,1);
               A_Norms = Eigen::MatrixXd::Ones(NR-row, 1);
               
               for(int i=row; i<NR; i++)
               {
                        //if(std::abs(A(i,xi_elim(i-row+1))) > Tol)
                        if(std::abs(A(i,xi_elim(i-row))) > Tol)
                        {
                                //A_Scaled.row(i) = A.row(i)/A(i,xi_elim(i-row+1));
                                A_Scaled.row(i) = A.row(i)/A(i,xi_elim(i-row));
                                //A_Norms(i,0) = A_Scaled.row(i).lpNorm<1>() - 1;
                                A_Norms(i-row,0) = A_Scaled.row(i).lpNorm<1>() - 1;
                        }
                        else
                        {
                                // Everything in row i from 1:NC is zero
                                A.block(i,0,1,NC).setZero();// = Eigen::MatrixXd::Zero()
                                A_Scaled.block(i,0,1,NC).setZero();
                                A_Norms(i-row,0) = Inf;
                        }
               }
               
               
//       	 std::vector< std::pair<double,int> > Anormvector;
//               for (int i=0; i<A_norms.rows(); i++)
//               {
//                       Anormvector.push_back(std::make_pair( A_Norms(i,0), i));
//               }
//               std::sort(Anormvector.begin(), Anormvector.end());
//               
//               A_Norms_ordered = A_Norms;

                // Instead of sorting A_Norms, it seems that the original algorithm only takes the first sorted index, which is equivalent to taking the index of the minimum of A_Norms(row:NR)
                //A_Norms.block(row,0,NR-row,1).row(0).minCoeff(&minrow);
                A_Norms.col(0).minCoeff(&minrow);
                //pivot_row = minrow+row-1;
                pivot_row = minrow+row;
                pivot_col = xi_elim(minrow);
                
                
                // Permute
                permuterow.setIdentity();
                permutecol.setIdentity();
                std::swap(permuterow.indices()[row], permuterow.indices()[pivot_row]);
                std::swap(permutecol.indices()[row], permutecol.indices()[pivot_col]);                
                
//                std::cout << "permuterow.indices() "<< std::endl << permuterow.indices() << std::endl << std::endl;                
//                std::cout << "nr "<< std::endl << nr << std::endl << std::endl;
//                std::cout << "permutecol.indices() "<< std::endl << permutecol.indices() << std::endl << std::endl;
//                std::cout << "nc "<< std::endl << nc << std::endl << std::endl;                                
//                std::cout << "A "<< std::endl << A << std::endl << std::endl;    
//                std::cout << "row_pivots "<< std::endl << row_pivots << std::endl << std::endl;  
//                std::cout << "col_pivots "<< std::endl << col_pivots << std::endl << std::endl;                                                              
//                printf("Cheguei aqui\n");   
                
                A = permuterow*A;
                row_pivots = row_pivots*permuterow;
                A = A*permutecol;
                col_pivots = col_pivots*permutecol;
                

                
                
                // Eliminate
                
                // CONTINUE HERE
                
                if(std::abs(A(row,row)) > Tol)
                {
                        A.block(row,row,1,nc-row) = A.block(row,row,1,nc-row)/A(row,row);
                        
                        // Eliminate downward
                        for(int i=row+1; i<nr; i++)
                        {
                                if(std::abs(A(i,row)) > Tol)
                                {
                                        A.row(i) = A.row(i) - A(i,row)*A.row(row);
                                }
                                else
                                {
                                        A(i,row) = 0;
                                }
                        }
                        
                        // Eliminate upward
                        for(int i=row-1; i>-1; i--)
                        {
                                if(std::abs(A(i,row)) > 1.0e-10)
                                {
                                        A.block(i,row,1,nc-row) = A.block(i,row,1,nc-row) - A(i,row)*A.block(row,row,1,nc-row);
                                }
                                else
                                {
                                        A(i,row) = 0;
                                }
                        }
                        
                }
                
                
                            
                        
        }
        
        return A;
        
}

cz czonotope::libczonrmzeroconstr(cz Z)
{
        /* Implementation of libCZon_RmZeroConstr.m in C++ */
        
        int flag = 0;
        
        
        int n = Z.G.rows();
        int ng = Z.G.cols();
        int nc = Z.A.rows();
        if(nc<1) // If Z is a zonotope
        {
                return Z;
        }
        
        
        double Epsilon = std::numeric_limits<double>::epsilon();
        double Tol = std::max(nc,ng)*Epsilon*Z.A.rowwise().lpNorm<1>().maxCoeff();          
        
        Eigen::MatrixXd RowMaxs(nc,1);
        RowMaxs = Z.A.cwiseAbs().rowwise().maxCoeff();
        
        for(int i=0; i<nc; i++)
        {
                if(RowMaxs(i)<=Tol)
                {
                        if(std::abs(Z.b(i)) > Tol)
                        {
                                // The set is empty
                                printf("Empty set detected in czonotope::libczonrmzeroconstr");
                                Z.c.resize(0,0);
                                Z.G.resize(0,0);
                                Z.A.resize(0,0);
                                Z.b.resize(0,0);                                                                                                                                                                        
                                return Z; 
                        }      
                        else
                        {
                                Z.A.block(i,0,nc-i-1,ng) = Z.A.bottomRows(nc-i-1);
                                Z.b.block(i,0,nc-i-1,1) = Z.b.bottomRows(nc-i-1);
                                RowMaxs.block(i,0,nc-i-1,1) = RowMaxs.bottomRows(nc-i-1);                                
                                
                                Z.A.conservativeResize(nc-1,ng);
                                Z.b.conservativeResize(nc-1,1);
                                RowMaxs.conservativeResize(nc-1,1);                                
                                
                                nc--;                           
                                i--;     
                                //matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) = matrix.bottomRows(numRows-rowToRemove);

                        }
                } 
        }
                        
        
        return Z;
}


void czonotope::cplexwrapper(Eigen::MatrixXd f, Eigen::MatrixXd A, Eigen::MatrixXd b, Eigen::MatrixXd Aeq, Eigen::MatrixXd beq, Eigen::MatrixXd lb, Eigen::MatrixXd ub, double& objvalue, Eigen::MatrixXd& xstar, int& exitflag)
{
        // TO DO
        
        // CPLEX wrapper: Concert C++ version

        // Calls CPLEX to solve min f'x subject to
        //
        //                      A*x <= b
        //                      Aeq*x = beq
        //                      lb <= x <= ub

        // f, A, b, Aeq, beq, lb and ub are MatrixXd objects of appropriate dimensions

        // Returns the optimal value of the objective function, the optimal value of x, and the exit flag from cplex

	// Code based on populatebynonzero

	IloEnv env;
	try {

                // Create CPLEX model, decision variables, constraints
		IloModel model(env);

		IloNumVarArray var(env);
		IloRangeArray con(env);

		// if (( argc != 2 )                         ||
		  // ( argv[1][0] != '-' )                 ||
		  // ( strchr ("rcn", argv[1][1]) == NULL )   ) {
		 // usage (argv[0]);
		 // throw(-1);
		// }

		//IloNumVarArray var(env);
		//IloRangeArray con(env);

                // Objective function
                IloObjective obj = IloMinimize(env);
                

                // Decision variables and lower/upper bounds
		// Populate by row code
		
		// switch (argv[1][1]) {
		 // case 'r':
			// populatebyrow (model, var, con);
			// break;
		 // case 'c':
			// populatebycolumn (model, var, con);
			// break;
		 // case 'n':
			// populatebynonzero (model, var, con);
			// break;
		// }		
		
                // Populate by nonzero code
		// populatebynonzero (IloModel model, IloNumVarArray x, IloRangeArray c)		
	  
                // IloEnv env = model.getEnv();

                //IloObjective obj = IloMaximize(env);

                if((lb.size()==0)||(ub.size()==0)) // No lower/upper bounds
                {
                        //printf("Entered [no lower/upper bounds]\n");
                        for(int j=0; j<f.rows(); j++)
                        {
                                //var.add(IloNumVar(env));
                                var.add(IloNumVar(env, -IloInfinity, IloInfinity)); // Corrected strange errors
                        }
                }
                else // With lower/upper bounds (only works if bounds are specified for EVERY decision variable)
                {
                        //printf("Entered [with lower/upper bounds]\n");
                        for(int j=0; j<f.rows(); j++)
                        {
                                var.add(IloNumVar(env, lb(j,0), ub(j,0)));
                                //std::cout << lb(j,0); printf(" "); std::cout << ub(j,0); printf("\n");
                        }
                }    
                
                
                
                // Objective function coefficients
                //printf("Entered [objective function coefficients]\n");                
                for(int j=0; j<f.rows(); j++)
                {
                        obj.setLinearCoef(var[j], f(j,0));
                        //std::cout << f(j,0); printf(" ");
                }                        
               // printf("\n");
   
   
                // Inequality constraints             
                for(int j=0; j<A.rows(); j++)
                {
                        //printf("Entered [inequality constraints]\n");  
                        con.add(IloRange(env, -IloInfinity, b(j,0)));
                        for(int i=0; i<A.cols(); i++)
                        {
                                                     
                               con[j].setLinearCoef(var[i], A(j,i)); 
                               //std::cout << A(j,i);  printf(" ");
                        }
                        //std::cout << b(j,0); printf("\n");
                } 
                
                
                // Equality constraints             
                for(int j=0; j<Aeq.rows(); j++)
                {
                        //printf("Entered [equality constraints]\n");                 
                        con.add(IloRange(env, beq(j,0), beq(j,0)));
                        for(int i=0; i<Aeq.cols(); i++)
                        {
                                                 
                               con[A.rows()+j].setLinearCoef(var[i], Aeq(j,i)); 
                               //std::cout << Aeq(j,i); printf(" ");
                        }
                        //std::cout << beq(j,0); printf("\n");                        
                }        
      
                
                model.add(obj);
                model.add(con);    
                
		IloCplex cplex(model);
		//cplex.exportModel("lpex1.lp");
		
		// Verbose = OFF
                cplex.setOut(env.getNullStream());


                exitflag = 0;
		// Optimize the problem and obtain solution.
		if ( !cplex.solve() ) {
		        env.error() << "Failed to optimize LP" << std::endl;
		        exitflag = -1;
		        //throw(-1);
		}  
		
                IloNumArray vals(env);
                cplex.getValues(vals, var);		
                
                objvalue = cplex.getObjValue(); // Optimal value of the objective function
                //std::cout << objvalue; std::cout << exitflag;
                for(int j=0; j<f.rows(); j++)
                {
                        xstar(j,0) = vals[j]; // Optimal solution
                }
		
//		IloNumArray vals(env);
//		env.out() << "Solution status = " << cplex.getStatus() << endl;
//		env.out() << "Solution value  = " << cplex.getObjValue() << endl;
//		cplex.getValues(vals, var);
//		env.out() << "Values        = " << vals << endl;
//		cplex.getSlacks(vals, con);
//		env.out() << "Slacks        = " << vals << endl;
//		cplex.getDuals(vals, con);
//		env.out() << "Duals         = " << vals << endl;
//		cplex.getReducedCosts(vals, var);
//		env.out() << "Reduced Costs = " << vals << endl;		                                   
                

//                c.add(IloRange(env, -IloInfinity, 20.0));
//                c.add(IloRange(env, -IloInfinity, 30.0));

//                x.add(IloNumVar(env, 0.0, 40.0));
//                x.add(IloNumVar(env));
//                x.add(IloNumVar(env));

//                obj.setLinearCoef(x[0], 1.0);
//                obj.setLinearCoef(x[1], 2.0);
//                obj.setLinearCoef(x[2], 3.0);

//                c[0].setLinearCoef(x[0], -1.0);
//                c[0].setLinearCoef(x[1],  1.0);
//                c[0].setLinearCoef(x[2],  1.0);
//                c[1].setLinearCoef(x[0],  1.0);
//                c[1].setLinearCoef(x[1], -3.0);
//                c[1].setLinearCoef(x[2],  1.0);

//                c[0].setName("c1");
//                c[1].setName("c2");

//                x[0].setName("x1");
//                x[1].setName("x2");
//                x[2].setName("x3");

//                model.add(obj);
//                model.add(c);  
//	  
//	  
//	  

//		IloCplex cplex(model);
//		cplex.exportModel("lpex1.lp");

//		// Optimize the problem and obtain solution.
//		if ( !cplex.solve() ) {
//		 env.error() << "Failed to optimize LP" << endl;
//		 throw(-1);
//		}

//		IloNumArray vals(env);
//		env.out() << "Solution status = " << cplex.getStatus() << endl;
//		env.out() << "Solution value  = " << cplex.getObjValue() << endl;
//		cplex.getValues(vals, var);
//		env.out() << "Values        = " << vals << endl;
//		cplex.getSlacks(vals, con);
//		env.out() << "Slacks        = " << vals << endl;
//		cplex.getDuals(vals, con);
//		env.out() << "Duals         = " << vals << endl;
//		cplex.getReducedCosts(vals, var);
//		env.out() << "Reduced Costs = " << vals << endl;
	}
	catch (IloException& e) {
		std::cerr << "Concert exception caught: " << e << std::endl;
	}
	catch (...) {
		std::cerr << "Unknown exception caught" << std::endl;
	}

	env.end();	

}


void czonotope::cplexwrapper2(Eigen::MatrixXd f, Eigen::MatrixXd A, Eigen::MatrixXd b, Eigen::MatrixXd Aeq, Eigen::MatrixXd beq, Eigen::MatrixXd LB, Eigen::MatrixXd UB, double& objvalue, Eigen::MatrixXd& xstar, int& exitflag)
{
        // TO DO
        
        // CPLEX Wrapper: Callable Libraries version

        // Calls CPLEX to solve min f'x subject to
        //
        //                      A*x <= b
        //                      Aeq*x = beq
        //                      LB <= x <= UB

        // f, A, b, Aeq, beq, LB and UB are MatrixXd objects of appropriate dimensions

        // Returns the optimal value of the objective function, the optimal value of x, and the exit flag from cplex
        
        exitflag = 0;
        
        int nof_x = f.rows();
        int nof_ineq = b.rows();
        int nof_eq = beq.rows();
        int nof_bounds = UB.rows();
        
        double* obj = f.data();
        double lb[nof_x];
        double ub[nof_x];    
        
        int i, j;        
        
        Eigen::MatrixXd Atotal(nof_ineq+nof_eq, nof_x);
        Eigen::MatrixXd btotal(nof_ineq+nof_eq, 1);
        if(nof_ineq!=0 && nof_eq==0)
        {
                Atotal = A;
                btotal = b;
        }
        else if(nof_ineq==0 && nof_eq!=0)
        {
                Atotal = Aeq;
                btotal = beq;
        }
        else
        {
                Atotal << A, Aeq;
                btotal << b, beq;
        }     
        
//        std::cout << "btotal " << btotal << std::endl;
        
        int totalrows = nof_ineq+nof_eq;
        
//        std::cout << "totalrows " << totalrows << std::endl;
        
        // Bounds on decision variables
        
//        double* obj = f.data();
//        double lb[nof_x];
//        double ub[nof_x];

        
        // Allocating arrays (one nested for loop)
        
        int      matbeg[nof_x];
        int      matind[nof_x*totalrows];        
        double*  matval = Atotal.data();
        double*  rhs = btotal.data();
        char     sense[totalrows];    
        
        if(nof_bounds==0)
        {
                for(j = 0; j<nof_x; j++)
                {
                        lb[j] = -CPX_INFBOUND;
                        ub[j] =  CPX_INFBOUND;
                        
                        matbeg[j] = j*totalrows;
                        
                        for(i=0; i<nof_ineq; i++)
                        {
                                sense[i] = 'L';
                                matind[j*totalrows + i] = i;                      
                        }
                        for(i=nof_ineq; i<totalrows; i++)
                        {
                                sense[i] = 'E';
                                matind[j*totalrows + i] = i;                      
                        }                              
                                          
                }
        }        
        else
        {
                
                for(j = 0; j<nof_x; j++)
                {
                        lb[j] =  LB(j,0);
                        ub[j] =  UB(j,0);

                        matbeg[j] = j*totalrows;
                        
                        for(i=0; i<nof_ineq; i++)
                        {
                                sense[i] = 'L';
                                matind[j*totalrows + i] = i;                      
                        }
                        for(i=nof_ineq; i<totalrows; i++)
                        {
                                sense[i] = 'E';
                                matind[j*totalrows + i] = i;                      
                        }                            
                }
        }                 
        
        // Allocating arrays (several for loops)
        
//        int      matbeg[nof_x];
//        int      matind[nof_x*totalrows];        
//        double*  matval = Atotal.data();
//        double*  rhs = btotal.data();
//        char     sense[totalrows];  
//        
//        if(nof_bounds==0)
//        {
//                for(j = 0; j<nof_x; j++)
//                {
//                        lb[j] = -CPX_INFBOUND;
//                        ub[j] =  CPX_INFBOUND;
//                }
//        }        
//        else
//        {
//                
//                for(j = 0; j<nof_x; j++)
//                {
//                        lb[j] =  LB(j,0);
//                        ub[j] =  UB(j,0);
//                }
//        }   
//        
//        for(j=0; j<nof_ineq; j++)
//        {
//                sense[j] = 'L';
//        }
//        for(j=nof_ineq; j<nof_ineq+nof_eq; j++)
//        {
//                sense[j] = 'E';
//        }      
//        
////        std::cout << "sense ";
////        for(j=0; j<totalrows; j++)
////        {
////                std::cout << sense[j] << " ";
////        }
////        std::cout << std::endl;
////        
////        std::cout << "rhs ";
////        for(j=0; j<totalrows; j++)
////        {
////                std::cout << rhs[j] << " ";
////        }
////        std::cout << std::endl;        
//        
////        
////        std::cout << "matbeg ";
//        for(j=0; j<nof_x; j++)
//        {
//                matbeg[j] = j*totalrows;
////                std::cout << matbeg[j] << " ";
//        }
////        std::cout << std::endl;        
//        
////        std::cout << "matind ";
//        for(j=0; j<nof_x; j++)
//        {
//                for(i=0; i<totalrows; i++)
//                {
//                        matind[j*totalrows + i] = i;
////                        std::cout << matind[j*totalrows+i] << " ";                        
//                }
//        }
////        std::cout << std::endl;   


                        
        
//        double* fdata = f.data();
//        double* Adata = A.data();
//        double* bdata = b.data();
//        double* Aeqdata = Aeq.data();
//        double* beqdata = beq.data();        

	// Code based on lpex1.c, populatebyrows and populatebycols
	
	int      solstat;
        double   objval;
        double   *x = NULL;
        double   *pi = NULL;
        double   *slack = NULL;
        double   *dj = NULL;
        
        CPXENVptr     env = NULL;
        CPXLPptr      lp = NULL;
        int           status = 0;
        int           cur_numrows, cur_numcols;    
        
        /* Initialize the CPLEX environment */

        env = CPXopenCPLEX (&status);

        /* If an error occurs, the status value indicates the reason for
        failure.  A call to CPXgeterrorstring will produce the text of
        the error message.  Note that CPXopenCPLEX produces no output,
        so the only way to see the cause of the error is to use
        CPXgeterrorstring.  For other CPLEX routines, the errors will
        be seen if the CPXPARAM_ScreenOutput indicator is set to CPX_ON.  */

        if ( env == NULL ) {
        char  errmsg[CPXMESSAGEBUFSIZE];
        fprintf (stderr, "Could not open CPLEX environment.\n");
        CPXgeterrorstring (env, status, errmsg);
        fprintf (stderr, "%s", errmsg);
        goto TERMINATE;
        }
        
        /* Turn on output to the screen */

        //status = CPXsetintparam (env, CPXPARAM_ScreenOutput, CPX_ON);
        status = CPXsetintparam (env, CPXPARAM_ScreenOutput, CPX_OFF);
        if ( status ) {
        fprintf (stderr, 
               "Failure to turn on screen indicator, error %d.\n", status);
        goto TERMINATE;
        }
        status = CPXsetintparam (env, CPX_PARAM_SCRIND, CPX_OFF);
        if ( status ) {
        fprintf (stderr, 
               "Failure to turn on screen indicator, error %d.\n", status);
        goto TERMINATE;
        }        
        

        /* Turn on data checking */

        status = CPXsetintparam (env, CPXPARAM_Read_DataCheck,
	                    CPX_DATACHECK_WARN);
        if ( status ) {
        fprintf (stderr, 
               "Failure to turn on data checking, error %d.\n", status);
        goto TERMINATE;
        }   
        
        /* Create the problem. */

        lp = CPXcreateprob (env, &status, "lpex1");

        /* A returned pointer of NULL may mean that not enough memory
        was available or there was some other problem.  In the case of 
        failure, an error message will have been written to the error 
        channel from inside CPLEX.  In this example, the setting of
        the parameter CPXPARAM_ScreenOutput causes the error message to
        appear on stdout.  */

        if ( lp == NULL ) {
        fprintf (stderr, "Failed to create LP.\n");
        goto TERMINATE;
        }             
        
        
//        Eigen::MatrixXd Atotal(nof_ineq+nof_eq, nof_x);
//        Eigen::MatrixXd btotal(nof_ineq+nof_eq, nof_x);
//        if(nof_ineq!=0 && nof_eq==0)
//        {
//                Atotal = A;
//                btotal = b;
//        }
//        else if(nof_ineq==0 && nof_eq!=0)
//        {
//                Atotal = Aeq;
//                btotal = beq;
//        }
//        else
//        {
//                Atotal << A, Aeq;
//                btotal << b, beq;
//        }
        
//        int totalrows = nof_ineq+nof_eq;
//        
//        
//        int      matbeg[nof_x];
//        int      matind[nof_x*totalrows];        
//        double*  matval = Atotal.data();
//        double*  rhs = btotal.data();
//        char     sense[totalrows];  

//        /* Now create the new rows.  First, populate the arrays. */

//        rowname[0] = "c1";
//        sense[0]   = 'L';
//        rhs[0]     = 20.0;

//        rowname[1] = "c2";
//        sense[1]   = 'L';
//        rhs[1]     = 30.0;

//        status = CPXnewrows (env, lp, NUMROWS, rhs, sense, NULL, rowname);
//        if ( status )   goto TERMINATE;

//        /* Now add the new columns.  First, populate the arrays. */

//        obj[0] = 1.0;      obj[1] = 2.0;           obj[2] = 3.0;

//        matbeg[0] = 0;     matbeg[1] = 2;          matbeg[2] = 4;

//        matind[0] = 0;     matind[2] = 0;          matind[4] = 0;
//        matval[0] = -1.0;  matval[2] = 1.0;        matval[4] = 1.0;

//        matind[1] = 1;     matind[3] = 1;          matind[5] = 1;
//        matval[1] = 1.0;   matval[3] = -3.0;       matval[5] = 1.0;

//        lb[0] = 0.0;       lb[1] = 0.0;            lb[2] = 0.0;
//        ub[0] = 40.0;      ub[1] = CPX_INFBOUND;   ub[2] = CPX_INFBOUND;

//        colname[0] = "x1"; colname[1] = "x2";      colname[2] = "x3";

//        status = CPXaddcols (env, lp, NUMCOLS, NUMNZ, obj, matbeg, matind,
//                        matval, lb, ub, colname);
//        if ( status )  goto TERMINATE;
        
//        for(j=0; j<nof_ineq; j++)
//        {
//                sense[j] = 'L';
//        }
//        for(j=nof_ineq; j<nof_ineq+nof_eq; j++)
//        {
//                sense[j] = 'E';
//        }      
//        
//        std::cout << "sense ";
//        for(j=0; j<totalrows; j++)
//        {
//                std::cout << sense[j] << " ";
//        }
//        std::cout << std::endl;
//        
//        std::cout << "rhs ";
//        for(j=0; j<totalrows; j++)
//        {
//                std::cout << rhs[j] << " ";
//        }
//        std::cout << std::endl;        
//        
//        
//        std::cout << "matbeg ";
//        for(j=0; j<nof_x; j++)
//        {
//                matbeg[j] = j*totalrows;
//                std::cout << matbeg[j] << " ";
//        }
//        std::cout << std::endl;        
//        
//        std::cout << "matind ";
//        for(j=0; j<nof_x; j++)
//        {
//                for(i=0; i<totalrows; i++)
//                {
//                        matind[j*totalrows + i] = i;
//                        std::cout << matind[j*totalrows+i] << " ";                        
//                }
//        }
//        std::cout << std::endl;        
        
        
        // Setup problem
        
        status = CPXchgobjsen (env, lp, CPX_MIN);  /* Problem is minimization */
        if ( status ){
        fprintf (stderr, "Failed to populate problem in: problem setup.\n");
        goto TERMINATE;
        }     
        
        
        // Create rows     
        
        status = CPXnewrows (env, lp, totalrows, rhs, sense, NULL, NULL);
        if ( status ){
        fprintf (stderr, "Failed to populate problem in: create rows.\n");
        goto TERMINATE;
        }        
        
        // Add cols
        
        status = CPXaddcols (env, lp, nof_x, nof_x*totalrows, obj, matbeg, matind,
                                matval, lb, ub, NULL);
        if ( status ){
        fprintf (stderr, "Failed to populate problem in: add cols.\n");
        goto TERMINATE;
        }
        

        /* Optimize the problem and obtain solution. */

        status = CPXlpopt (env, lp);
        if ( status ) {
        fprintf (stderr, "Failed to optimize LP.\n");
        exitflag = -1;
        goto TERMINATE;
        }

        /* The size of the problem should be obtained by asking CPLEX what
        the actual size is, rather than using sizes from when the problem
        was built.  cur_numrows and cur_numcols store the current number 
        of rows and columns, respectively.  */

        cur_numrows = CPXgetnumrows (env, lp);
        cur_numcols = CPXgetnumcols (env, lp);

        x = (double *) malloc (cur_numcols * sizeof(double));
        slack = (double *) malloc (cur_numrows * sizeof(double));
        dj = (double *) malloc (cur_numcols * sizeof(double));
        pi = (double *) malloc (cur_numrows * sizeof(double));

        if ( x     == NULL ||
        slack == NULL ||
        dj    == NULL ||
        pi    == NULL   ) {
        status = CPXERR_NO_MEMORY;
        fprintf (stderr, "Could not allocate memory for solution.\n");
        goto TERMINATE;
        }

        status = CPXsolution (env, lp, &solstat, &objval, x, pi, slack, dj);
        if ( status ) {
        exitflag = -1;
        fprintf (stderr, "Failed to obtain solution.\n");
        goto TERMINATE;
        }

//        /* Write the output to the screen. */

//        printf ("\nSolution status = %d\n", solstat);
//        printf ("Solution value  = %f\n\n", objval);

//        for (i = 0; i < cur_numrows; i++) {
//        printf ("Row %d:  Slack = %10f  Pi = %10f\n", i, slack[i], pi[i]);
//        }

//        for (j = 0; j < cur_numcols; j++) {
//        printf ("Column %d:  Value = %10f  Reduced cost = %10f\n",
//              j, x[j], dj[j]);
//        }

//        /* Finally, write a copy of the problem to a file. */

//        status = CPXwriteprob (env, lp, "lpex1.lp", NULL);
//        if ( status ) {
//        fprintf (stderr, "Failed to write LP to disk.\n");
//        goto TERMINATE;
//        }

        objvalue = objval;
        for(int j=0; j<nof_x; j++)
        {
                xstar(j,0) = x[j]; // Optimal solution
        }
        
//        std::cout << "rhs ";
//        for(j=0; j<totalrows; j++)
//        {
//                std::cout << rhs[j] << " ";
//        }
//        std::cout << std::endl;         

        TERMINATE:

        /* Free up the solution */

        free_and_null ((char **) &x);
        free_and_null ((char **) &slack);
        free_and_null ((char **) &dj);
        free_and_null ((char **) &pi);

        /* Free up the problem as allocated by CPXcreateprob, if necessary */

        if ( lp != NULL ) {
        status = CPXfreeprob (env, &lp);
        if ( status ) {
         fprintf (stderr, "CPXfreeprob failed, error code %d.\n", status);
        }
        }

        /* Free up the CPLEX environment, if necessary */

        if ( env != NULL ) {
        status = CPXcloseCPLEX (&env);

        /* Note that CPXcloseCPLEX produces no output,
         so the only way to see the cause of the error is to use
         CPXgeterrorstring.  For other CPLEX routines, the errors will
         be seen if the CPXPARAM_ScreenOutput indicator is set to CPX_ON. */

        if ( status ) {
         char  errmsg[CPXMESSAGEBUFSIZE];
         fprintf (stderr, "Could not close CPLEX environment.\n");
         CPXgeterrorstring (env, status, errmsg);
         fprintf (stderr, "%s", errmsg);
        }
        }
                  
                   

}

void czonotope::cplexwrapper2minmax(Eigen::MatrixXd f, Eigen::MatrixXd A, Eigen::MatrixXd b, Eigen::MatrixXd Aeq, Eigen::MatrixXd beq, Eigen::MatrixXd LB, Eigen::MatrixXd UB, double& objvaluemin, double& objvaluemax, Eigen::MatrixXd& xstarmin, Eigen::MatrixXd& xstarmax, int& exitflag)        
{
        // TO DO
        
        // CPLEX Wrapper: Callable Libraries version

        // Calls CPLEX to solve min f'x subject to
        //
        //                      A*x <= b
        //                      Aeq*x = beq
        //                      LB <= x <= UB

        // f, A, b, Aeq, beq, LB and UB are MatrixXd objects of appropriate dimensions

        // Returns the optimal value of the objective function, the optimal value of x, and the exit flag from cplex
        
        exitflag = 0;
        
        int nof_x = f.rows();
        int nof_ineq = b.rows();
        int nof_eq = beq.rows();
        int nof_bounds = UB.rows();
        
        double* obj = f.data();
        double lb[nof_x];
        double ub[nof_x];    
        
        int i, j;        
        
        Eigen::MatrixXd Atotal(nof_ineq+nof_eq, nof_x);
        Eigen::MatrixXd btotal(nof_ineq+nof_eq, 1);
        if(nof_ineq!=0 && nof_eq==0)
        {
                Atotal = A;
                btotal = b;
        }
        else if(nof_ineq==0 && nof_eq!=0)
        {
                Atotal = Aeq;
                btotal = beq;
        }
        else
        {
                Atotal << A, Aeq;
                btotal << b, beq;
        }     
        
//        std::cout << "btotal " << btotal << std::endl;
        
        int totalrows = nof_ineq+nof_eq;
        
//        std::cout << "totalrows " << totalrows << std::endl;
        
        // Bounds on decision variables
        
//        double* obj = f.data();
//        double lb[nof_x];
//        double ub[nof_x];

        
        // Allocating arrays (one nested for loop)
        
        int      matbeg[nof_x];
        int      matind[nof_x*totalrows];        
        double*  matval = Atotal.data();
        double*  rhs = btotal.data();
        char     sense[totalrows];    
        
        if(nof_bounds==0)
        {
                for(j = 0; j<nof_x; j++)
                {
                        lb[j] = -CPX_INFBOUND;
                        ub[j] =  CPX_INFBOUND;
                        
                        matbeg[j] = j*totalrows;
                        
                        for(i=0; i<nof_ineq; i++)
                        {
                                sense[i] = 'L';
                                matind[j*totalrows + i] = i;                      
                        }
                        for(i=nof_ineq; i<totalrows; i++)
                        {
                                sense[i] = 'E';
                                matind[j*totalrows + i] = i;                      
                        }                              
                                          
                }
        }        
        else
        {
                
                for(j = 0; j<nof_x; j++)
                {
                        lb[j] =  LB(j,0);
                        ub[j] =  UB(j,0);

                        matbeg[j] = j*totalrows;
                        
                        for(i=0; i<nof_ineq; i++)
                        {
                                sense[i] = 'L';
                                matind[j*totalrows + i] = i;                      
                        }
                        for(i=nof_ineq; i<totalrows; i++)
                        {
                                sense[i] = 'E';
                                matind[j*totalrows + i] = i;                      
                        }                            
                }
        }                 
        
        // Allocating arrays (several for loops)
        
//        int      matbeg[nof_x];
//        int      matind[nof_x*totalrows];        
//        double*  matval = Atotal.data();
//        double*  rhs = btotal.data();
//        char     sense[totalrows];  
//        
//        if(nof_bounds==0)
//        {
//                for(j = 0; j<nof_x; j++)
//                {
//                        lb[j] = -CPX_INFBOUND;
//                        ub[j] =  CPX_INFBOUND;
//                }
//        }        
//        else
//        {
//                
//                for(j = 0; j<nof_x; j++)
//                {
//                        lb[j] =  LB(j,0);
//                        ub[j] =  UB(j,0);
//                }
//        }   
//        
//        for(j=0; j<nof_ineq; j++)
//        {
//                sense[j] = 'L';
//        }
//        for(j=nof_ineq; j<nof_ineq+nof_eq; j++)
//        {
//                sense[j] = 'E';
//        }      
//        
////        std::cout << "sense ";
////        for(j=0; j<totalrows; j++)
////        {
////                std::cout << sense[j] << " ";
////        }
////        std::cout << std::endl;
////        
////        std::cout << "rhs ";
////        for(j=0; j<totalrows; j++)
////        {
////                std::cout << rhs[j] << " ";
////        }
////        std::cout << std::endl;        
//        
////        
////        std::cout << "matbeg ";
//        for(j=0; j<nof_x; j++)
//        {
//                matbeg[j] = j*totalrows;
////                std::cout << matbeg[j] << " ";
//        }
////        std::cout << std::endl;        
//        
////        std::cout << "matind ";
//        for(j=0; j<nof_x; j++)
//        {
//                for(i=0; i<totalrows; i++)
//                {
//                        matind[j*totalrows + i] = i;
////                        std::cout << matind[j*totalrows+i] << " ";                        
//                }
//        }
////        std::cout << std::endl;   


                        
        
//        double* fdata = f.data();
//        double* Adata = A.data();
//        double* bdata = b.data();
//        double* Aeqdata = Aeq.data();
//        double* beqdata = beq.data();        

	// Code based on lpex1.c, populatebyrows and populatebycols
	
	int      solstat;
        double   objval;
        double   *x = NULL;
        double   *pi = NULL;
        double   *slack = NULL;
        double   *dj = NULL;
        
        CPXENVptr     env = NULL;
        CPXLPptr      lp = NULL;
        int           status = 0;
        int           cur_numrows, cur_numcols;    
        
        /* Initialize the CPLEX environment */

        env = CPXopenCPLEX (&status);

        /* If an error occurs, the status value indicates the reason for
        failure.  A call to CPXgeterrorstring will produce the text of
        the error message.  Note that CPXopenCPLEX produces no output,
        so the only way to see the cause of the error is to use
        CPXgeterrorstring.  For other CPLEX routines, the errors will
        be seen if the CPXPARAM_ScreenOutput indicator is set to CPX_ON.  */

        if ( env == NULL ) {
        char  errmsg[CPXMESSAGEBUFSIZE];
        fprintf (stderr, "Could not open CPLEX environment.\n");
        CPXgeterrorstring (env, status, errmsg);
        fprintf (stderr, "%s", errmsg);
        goto TERMINATE;
        }
        
        /* Turn on output to the screen */

        //status = CPXsetintparam (env, CPXPARAM_ScreenOutput, CPX_ON);
        status = CPXsetintparam (env, CPXPARAM_ScreenOutput, CPX_OFF);
        if ( status ) {
        fprintf (stderr, 
               "Failure to turn on screen indicator, error %d.\n", status);
        goto TERMINATE;
        }
        status = CPXsetintparam (env, CPX_PARAM_SCRIND, CPX_OFF);
        if ( status ) {
        fprintf (stderr, 
               "Failure to turn on screen indicator, error %d.\n", status);
        goto TERMINATE;
        }        
        

        /* Turn on data checking */

        status = CPXsetintparam (env, CPXPARAM_Read_DataCheck,
	                    CPX_DATACHECK_WARN);
        if ( status ) {
        fprintf (stderr, 
               "Failure to turn on data checking, error %d.\n", status);
        goto TERMINATE;
        }   
        
        /* Create the problem. */

        lp = CPXcreateprob (env, &status, "lpex1");

        /* A returned pointer of NULL may mean that not enough memory
        was available or there was some other problem.  In the case of 
        failure, an error message will have been written to the error 
        channel from inside CPLEX.  In this example, the setting of
        the parameter CPXPARAM_ScreenOutput causes the error message to
        appear on stdout.  */

        if ( lp == NULL ) {
        fprintf (stderr, "Failed to create LP.\n");
        goto TERMINATE;
        }             
        
        
//        Eigen::MatrixXd Atotal(nof_ineq+nof_eq, nof_x);
//        Eigen::MatrixXd btotal(nof_ineq+nof_eq, nof_x);
//        if(nof_ineq!=0 && nof_eq==0)
//        {
//                Atotal = A;
//                btotal = b;
//        }
//        else if(nof_ineq==0 && nof_eq!=0)
//        {
//                Atotal = Aeq;
//                btotal = beq;
//        }
//        else
//        {
//                Atotal << A, Aeq;
//                btotal << b, beq;
//        }
        
//        int totalrows = nof_ineq+nof_eq;
//        
//        
//        int      matbeg[nof_x];
//        int      matind[nof_x*totalrows];        
//        double*  matval = Atotal.data();
//        double*  rhs = btotal.data();
//        char     sense[totalrows];  

//        /* Now create the new rows.  First, populate the arrays. */

//        rowname[0] = "c1";
//        sense[0]   = 'L';
//        rhs[0]     = 20.0;

//        rowname[1] = "c2";
//        sense[1]   = 'L';
//        rhs[1]     = 30.0;

//        status = CPXnewrows (env, lp, NUMROWS, rhs, sense, NULL, rowname);
//        if ( status )   goto TERMINATE;

//        /* Now add the new columns.  First, populate the arrays. */

//        obj[0] = 1.0;      obj[1] = 2.0;           obj[2] = 3.0;

//        matbeg[0] = 0;     matbeg[1] = 2;          matbeg[2] = 4;

//        matind[0] = 0;     matind[2] = 0;          matind[4] = 0;
//        matval[0] = -1.0;  matval[2] = 1.0;        matval[4] = 1.0;

//        matind[1] = 1;     matind[3] = 1;          matind[5] = 1;
//        matval[1] = 1.0;   matval[3] = -3.0;       matval[5] = 1.0;

//        lb[0] = 0.0;       lb[1] = 0.0;            lb[2] = 0.0;
//        ub[0] = 40.0;      ub[1] = CPX_INFBOUND;   ub[2] = CPX_INFBOUND;

//        colname[0] = "x1"; colname[1] = "x2";      colname[2] = "x3";

//        status = CPXaddcols (env, lp, NUMCOLS, NUMNZ, obj, matbeg, matind,
//                        matval, lb, ub, colname);
//        if ( status )  goto TERMINATE;
        
//        for(j=0; j<nof_ineq; j++)
//        {
//                sense[j] = 'L';
//        }
//        for(j=nof_ineq; j<nof_ineq+nof_eq; j++)
//        {
//                sense[j] = 'E';
//        }      
//        
//        std::cout << "sense ";
//        for(j=0; j<totalrows; j++)
//        {
//                std::cout << sense[j] << " ";
//        }
//        std::cout << std::endl;
//        
//        std::cout << "rhs ";
//        for(j=0; j<totalrows; j++)
//        {
//                std::cout << rhs[j] << " ";
//        }
//        std::cout << std::endl;        
//        
//        
//        std::cout << "matbeg ";
//        for(j=0; j<nof_x; j++)
//        {
//                matbeg[j] = j*totalrows;
//                std::cout << matbeg[j] << " ";
//        }
//        std::cout << std::endl;        
//        
//        std::cout << "matind ";
//        for(j=0; j<nof_x; j++)
//        {
//                for(i=0; i<totalrows; i++)
//                {
//                        matind[j*totalrows + i] = i;
//                        std::cout << matind[j*totalrows+i] << " ";                        
//                }
//        }
//        std::cout << std::endl;        
        
        
        // Minimization
        
        // Setup problem
        
        status = CPXchgobjsen (env, lp, CPX_MIN);  /* Problem is minimization */
        if ( status ){
        fprintf (stderr, "Failed to populate problem in: problem setup (min).\n");
        goto TERMINATE;
        }     
        
        
        // Create rows     
        
        status = CPXnewrows (env, lp, totalrows, rhs, sense, NULL, NULL);
        if ( status ){
        fprintf (stderr, "Failed to populate problem in: create rows.\n");
        goto TERMINATE;
        }        
        
        // Add cols
        
        status = CPXaddcols (env, lp, nof_x, nof_x*totalrows, obj, matbeg, matind,
                                matval, lb, ub, NULL);
        if ( status ){
        fprintf (stderr, "Failed to populate problem in: add cols.\n");
        goto TERMINATE;
        }
        

        /* Optimize the problem and obtain solution. */

        status = CPXlpopt (env, lp);
        if ( status ) {
        fprintf (stderr, "Failed to optimize LP.\n");
        exitflag = -1;
        goto TERMINATE;
        }

        /* The size of the problem should be obtained by asking CPLEX what
        the actual size is, rather than using sizes from when the problem
        was built.  cur_numrows and cur_numcols store the current number 
        of rows and columns, respectively.  */

        cur_numrows = CPXgetnumrows (env, lp);
        cur_numcols = CPXgetnumcols (env, lp);

        x = (double *) malloc (cur_numcols * sizeof(double));
        slack = (double *) malloc (cur_numrows * sizeof(double));
        dj = (double *) malloc (cur_numcols * sizeof(double));
        pi = (double *) malloc (cur_numrows * sizeof(double));

        if ( x     == NULL ||
        slack == NULL ||
        dj    == NULL ||
        pi    == NULL   ) {
        status = CPXERR_NO_MEMORY;
        fprintf (stderr, "Could not allocate memory for solution.\n");
        goto TERMINATE;
        }

        status = CPXsolution (env, lp, &solstat, &objval, x, pi, slack, dj);
        if ( status ) {
        exitflag = -1;
        fprintf (stderr, "Failed to obtain solution.\n");
        goto TERMINATE;
        }

//        /* Write the output to the screen. */

//        printf ("\nSolution status = %d\n", solstat);
//        printf ("Solution value  = %f\n\n", objval);

//        for (i = 0; i < cur_numrows; i++) {
//        printf ("Row %d:  Slack = %10f  Pi = %10f\n", i, slack[i], pi[i]);
//        }

//        for (j = 0; j < cur_numcols; j++) {
//        printf ("Column %d:  Value = %10f  Reduced cost = %10f\n",
//              j, x[j], dj[j]);
//        }

//        /* Finally, write a copy of the problem to a file. */

//        status = CPXwriteprob (env, lp, "lpex1.lp", NULL);
//        if ( status ) {
//        fprintf (stderr, "Failed to write LP to disk.\n");
//        goto TERMINATE;
//        }

        objvaluemin = objval;
        for(int j=0; j<nof_x; j++)
        {
                xstarmin(j,0) = x[j]; // Optimal solution
        }
        

       
        // Maximization
        
        // Setup problem
        
        status = CPXchgobjsen (env, lp, CPX_MAX);  /* Problem is maximization */
        if ( status ){
        fprintf (stderr, "Failed to populate problem in: problem setup (max).\n");
        goto TERMINATE;
        }          
        
        
        /* Optimize the problem and obtain solution. */

        status = CPXlpopt (env, lp);
        if ( status ) {
        fprintf (stderr, "Failed to optimize LP.\n");
        exitflag = -1;
        goto TERMINATE;
        }

        /* The size of the problem should be obtained by asking CPLEX what
        the actual size is, rather than using sizes from when the problem
        was built.  cur_numrows and cur_numcols store the current number 
        of rows and columns, respectively.  */

//        cur_numrows = CPXgetnumrows (env, lp);
//        cur_numcols = CPXgetnumcols (env, lp);

//        x = (double *) malloc (cur_numcols * sizeof(double));
//        slack = (double *) malloc (cur_numrows * sizeof(double));
//        dj = (double *) malloc (cur_numcols * sizeof(double));
//        pi = (double *) malloc (cur_numrows * sizeof(double));

//        if ( x     == NULL ||
//        slack == NULL ||
//        dj    == NULL ||
//        pi    == NULL   ) {
//        status = CPXERR_NO_MEMORY;
//        fprintf (stderr, "Could not allocate memory for solution.\n");
//        goto TERMINATE;
//        }

        status = CPXsolution (env, lp, &solstat, &objval, x, pi, slack, dj);
        if ( status ) {
        exitflag = -1;
        fprintf (stderr, "Failed to obtain solution.\n");
        goto TERMINATE;
        }      
        
        objvaluemax = objval;
        for(int j=0; j<nof_x; j++)
        {
                xstarmax(j,0) = x[j]; // Optimal solution
        }          
        
        
//        std::cout << "rhs ";
//        for(j=0; j<totalrows; j++)
//        {
//                std::cout << rhs[j] << " ";
//        }
//        std::cout << std::endl;         

        TERMINATE:

        /* Free up the solution */

        free_and_null ((char **) &x);
        free_and_null ((char **) &slack);
        free_and_null ((char **) &dj);
        free_and_null ((char **) &pi);

        /* Free up the problem as allocated by CPXcreateprob, if necessary */

        if ( lp != NULL ) {
        status = CPXfreeprob (env, &lp);
        if ( status ) {
         fprintf (stderr, "CPXfreeprob failed, error code %d.\n", status);
        }
        }

        /* Free up the CPLEX environment, if necessary */

        if ( env != NULL ) {
        status = CPXcloseCPLEX (&env);

        /* Note that CPXcloseCPLEX produces no output,
         so the only way to see the cause of the error is to use
         CPXgeterrorstring.  For other CPLEX routines, the errors will
         be seen if the CPXPARAM_ScreenOutput indicator is set to CPX_ON. */

        if ( status ) {
         char  errmsg[CPXMESSAGEBUFSIZE];
         fprintf (stderr, "Could not close CPLEX environment.\n");
         CPXgeterrorstring (env, status, errmsg);
         fprintf (stderr, "%s", errmsg);
        }
        }
                  
                   

}

void czonotope::cplexwrapper2minmax_manyf_fixconst(Eigen::MatrixXd f, Eigen::MatrixXd A, Eigen::MatrixXd b, Eigen::MatrixXd Aeq, Eigen::MatrixXd beq, Eigen::MatrixXd LB, Eigen::MatrixXd UB, Eigen::MatrixXd& objvaluemin, Eigen::MatrixXd& objvaluemax, int& exitflag)
{
        
        // TO DO
        
        // CPLEX Wrapper: Callable Libraries version

        // Calls CPLEX to solve min f'x subject to
        //
        //                      A*x <= b
        //                      Aeq*x = beq
        //                      LB <= x <= UB

        // f, A, b, Aeq, beq, LB and UB are MatrixXd objects of appropriate dimensions

        // Returns the optimal value of the objective function, the optimal value of x, and the exit flag from cplex
        
        exitflag = 0;
        
        int nof_x = f.rows();
        int nof_LPs = f.cols();
        int nof_ineq = b.rows();
        int nof_eq = beq.rows();
        int nof_bounds = UB.rows();
        
        double* obj = f.col(0).data();        
        double lb[nof_x];
        double ub[nof_x];    
        
        int i, j;        
        
        Eigen::MatrixXd Atotal(nof_ineq+nof_eq, nof_x);
        Eigen::MatrixXd btotal(nof_ineq+nof_eq, 1);
        if(nof_ineq!=0 && nof_eq==0)
        {
                Atotal = A;
                btotal = b;
        }
        else if(nof_ineq==0 && nof_eq!=0)
        {
                Atotal = Aeq;
                btotal = beq;
        }
        else
        {
                Atotal << A, Aeq;
                btotal << b, beq;
        }     
        
//        std::cout << "btotal " << btotal << std::endl;
        
        int totalrows = nof_ineq+nof_eq;
        
//        std::cout << "totalrows " << totalrows << std::endl;
        
        // Bounds on decision variables
        
//        double* obj = f.data();
//        double lb[nof_x];
//        double ub[nof_x];

        
        // Allocating arrays (one nested for loop)
        
        int      matbeg[nof_x];
        int      matind[nof_x*totalrows];        
        double*  matval = Atotal.data();
        double*  rhs = btotal.data();
        char     sense[totalrows]; 
        int      objind[nof_x];
        
        if(nof_bounds==0)
        {
                for(j = 0; j<nof_x; j++)
                {
                
                        lb[j] = -CPX_INFBOUND;
                        ub[j] =  CPX_INFBOUND;
                        
                        objind[j] = j;
                        
                        matbeg[j] = j*totalrows;
                        
                        for(i=0; i<nof_ineq; i++)
                        {
                                sense[i] = 'L';
                                matind[j*totalrows + i] = i;                      
                        }
                        for(i=nof_ineq; i<totalrows; i++)
                        {
                                sense[i] = 'E';
                                matind[j*totalrows + i] = i;                      
                        }                              
                                          
                }
        }        
        else
        {
                
                for(j = 0; j<nof_x; j++)
                {
                        lb[j] =  LB(j,0);
                        ub[j] =  UB(j,0);
                        
                        objind[j] = j;                        

                        matbeg[j] = j*totalrows;
                        
                        for(i=0; i<nof_ineq; i++)
                        {
                                sense[i] = 'L';
                                matind[j*totalrows + i] = i;                      
                        }
                        for(i=nof_ineq; i<totalrows; i++)
                        {
                                sense[i] = 'E';
                                matind[j*totalrows + i] = i;                      
                        }                            
                }
        }                 
        
        // Allocating arrays (several for loops)
        
//        int      matbeg[nof_x];
//        int      matind[nof_x*totalrows];        
//        double*  matval = Atotal.data();
//        double*  rhs = btotal.data();
//        char     sense[totalrows];  
//        
//        if(nof_bounds==0)
//        {
//                for(j = 0; j<nof_x; j++)
//                {
//                        lb[j] = -CPX_INFBOUND;
//                        ub[j] =  CPX_INFBOUND;
//                }
//        }        
//        else
//        {
//                
//                for(j = 0; j<nof_x; j++)
//                {
//                        lb[j] =  LB(j,0);
//                        ub[j] =  UB(j,0);
//                }
//        }   
//        
//        for(j=0; j<nof_ineq; j++)
//        {
//                sense[j] = 'L';
//        }
//        for(j=nof_ineq; j<nof_ineq+nof_eq; j++)
//        {
//                sense[j] = 'E';
//        }      
//        
////        std::cout << "sense ";
////        for(j=0; j<totalrows; j++)
////        {
////                std::cout << sense[j] << " ";
////        }
////        std::cout << std::endl;
////        
////        std::cout << "rhs ";
////        for(j=0; j<totalrows; j++)
////        {
////                std::cout << rhs[j] << " ";
////        }
////        std::cout << std::endl;        
//        
////        
////        std::cout << "matbeg ";
//        for(j=0; j<nof_x; j++)
//        {
//                matbeg[j] = j*totalrows;
////                std::cout << matbeg[j] << " ";
//        }
////        std::cout << std::endl;        
//        
////        std::cout << "matind ";
//        for(j=0; j<nof_x; j++)
//        {
//                for(i=0; i<totalrows; i++)
//                {
//                        matind[j*totalrows + i] = i;
////                        std::cout << matind[j*totalrows+i] << " ";                        
//                }
//        }
////        std::cout << std::endl;   


                        
        
//        double* fdata = f.data();
//        double* Adata = A.data();
//        double* bdata = b.data();
//        double* Aeqdata = Aeq.data();
//        double* beqdata = beq.data();        

	// Code based on lpex1.c, populatebyrows and populatebycols
	
	int      solstat;
        double   objval;
        double   *x = NULL;
        double   *pi = NULL;
        double   *slack = NULL;
        double   *dj = NULL;
        
        CPXENVptr     env = NULL;
        CPXLPptr      lp = NULL;
        int           status = 0;
        int           cur_numrows, cur_numcols;    
        
        /* Initialize the CPLEX environment */

        env = CPXopenCPLEX (&status);

        /* If an error occurs, the status value indicates the reason for
        failure.  A call to CPXgeterrorstring will produce the text of
        the error message.  Note that CPXopenCPLEX produces no output,
        so the only way to see the cause of the error is to use
        CPXgeterrorstring.  For other CPLEX routines, the errors will
        be seen if the CPXPARAM_ScreenOutput indicator is set to CPX_ON.  */

        if ( env == NULL ) {
        char  errmsg[CPXMESSAGEBUFSIZE];
        fprintf (stderr, "Could not open CPLEX environment.\n");
        CPXgeterrorstring (env, status, errmsg);
        fprintf (stderr, "%s", errmsg);
        goto TERMINATE;
        }
        
        /* Turn on output to the screen */

        //status = CPXsetintparam (env, CPXPARAM_ScreenOutput, CPX_ON);
        status = CPXsetintparam (env, CPXPARAM_ScreenOutput, CPX_OFF);
        if ( status ) {
        fprintf (stderr, 
               "Failure to turn on screen indicator, error %d.\n", status);
        goto TERMINATE;
        }
        status = CPXsetintparam (env, CPX_PARAM_SCRIND, CPX_OFF);
        if ( status ) {
        fprintf (stderr, 
               "Failure to turn on screen indicator, error %d.\n", status);
        goto TERMINATE;
        }        
        

        /* Turn on data checking */

        status = CPXsetintparam (env, CPXPARAM_Read_DataCheck,
	                    CPX_DATACHECK_WARN);
        if ( status ) {
        fprintf (stderr, 
               "Failure to turn on data checking, error %d.\n", status);
        goto TERMINATE;
        }   
        
        /* Create the problem. */

        lp = CPXcreateprob (env, &status, "lpex1");

        /* A returned pointer of NULL may mean that not enough memory
        was available or there was some other problem.  In the case of 
        failure, an error message will have been written to the error 
        channel from inside CPLEX.  In this example, the setting of
        the parameter CPXPARAM_ScreenOutput causes the error message to
        appear on stdout.  */

        if ( lp == NULL ) {
        fprintf (stderr, "Failed to create LP.\n");
        goto TERMINATE;
        }             
        
        
//        Eigen::MatrixXd Atotal(nof_ineq+nof_eq, nof_x);
//        Eigen::MatrixXd btotal(nof_ineq+nof_eq, nof_x);
//        if(nof_ineq!=0 && nof_eq==0)
//        {
//                Atotal = A;
//                btotal = b;
//        }
//        else if(nof_ineq==0 && nof_eq!=0)
//        {
//                Atotal = Aeq;
//                btotal = beq;
//        }
//        else
//        {
//                Atotal << A, Aeq;
//                btotal << b, beq;
//        }
        
//        int totalrows = nof_ineq+nof_eq;
//        
//        
//        int      matbeg[nof_x];
//        int      matind[nof_x*totalrows];        
//        double*  matval = Atotal.data();
//        double*  rhs = btotal.data();
//        char     sense[totalrows];  

//        /* Now create the new rows.  First, populate the arrays. */

//        rowname[0] = "c1";
//        sense[0]   = 'L';
//        rhs[0]     = 20.0;

//        rowname[1] = "c2";
//        sense[1]   = 'L';
//        rhs[1]     = 30.0;

//        status = CPXnewrows (env, lp, NUMROWS, rhs, sense, NULL, rowname);
//        if ( status )   goto TERMINATE;

//        /* Now add the new columns.  First, populate the arrays. */

//        obj[0] = 1.0;      obj[1] = 2.0;           obj[2] = 3.0;

//        matbeg[0] = 0;     matbeg[1] = 2;          matbeg[2] = 4;

//        matind[0] = 0;     matind[2] = 0;          matind[4] = 0;
//        matval[0] = -1.0;  matval[2] = 1.0;        matval[4] = 1.0;

//        matind[1] = 1;     matind[3] = 1;          matind[5] = 1;
//        matval[1] = 1.0;   matval[3] = -3.0;       matval[5] = 1.0;

//        lb[0] = 0.0;       lb[1] = 0.0;            lb[2] = 0.0;
//        ub[0] = 40.0;      ub[1] = CPX_INFBOUND;   ub[2] = CPX_INFBOUND;

//        colname[0] = "x1"; colname[1] = "x2";      colname[2] = "x3";

//        status = CPXaddcols (env, lp, NUMCOLS, NUMNZ, obj, matbeg, matind,
//                        matval, lb, ub, colname);
//        if ( status )  goto TERMINATE;
        
//        for(j=0; j<nof_ineq; j++)
//        {
//                sense[j] = 'L';
//        }
//        for(j=nof_ineq; j<nof_ineq+nof_eq; j++)
//        {
//                sense[j] = 'E';
//        }      
//        
//        std::cout << "sense ";
//        for(j=0; j<totalrows; j++)
//        {
//                std::cout << sense[j] << " ";
//        }
//        std::cout << std::endl;
//        
//        std::cout << "rhs ";
//        for(j=0; j<totalrows; j++)
//        {
//                std::cout << rhs[j] << " ";
//        }
//        std::cout << std::endl;        
//        
//        
//        std::cout << "matbeg ";
//        for(j=0; j<nof_x; j++)
//        {
//                matbeg[j] = j*totalrows;
//                std::cout << matbeg[j] << " ";
//        }
//        std::cout << std::endl;        
//        
//        std::cout << "matind ";
//        for(j=0; j<nof_x; j++)
//        {
//                for(i=0; i<totalrows; i++)
//                {
//                        matind[j*totalrows + i] = i;
//                        std::cout << matind[j*totalrows+i] << " ";                        
//                }
//        }
//        std::cout << std::endl;        
        
        
        // Minimization
        
        // Setup problem
        
        status = CPXchgobjsen (env, lp, CPX_MIN);  /* Problem is minimization */
        if ( status ){
        fprintf (stderr, "Failed to populate problem in: problem setup (min).\n");
        goto TERMINATE;
        }     
        
        
        // Create rows     
        
        status = CPXnewrows (env, lp, totalrows, rhs, sense, NULL, NULL);
        if ( status ){
        fprintf (stderr, "Failed to populate problem in: create rows.\n");
        goto TERMINATE;
        }        
        
        // Add cols
        
        status = CPXaddcols (env, lp, nof_x, nof_x*totalrows, obj, matbeg, matind,
                                matval, lb, ub, NULL);
        if ( status ){
        fprintf (stderr, "Failed to populate problem in: add cols.\n");
        goto TERMINATE;
        }
        

        /* Optimize the problem and obtain solution. */

        status = CPXlpopt (env, lp);
        if ( status ) {
        fprintf (stderr, "Failed to optimize LP.\n");
        exitflag = -1;
        goto TERMINATE;
        }

        /* The size of the problem should be obtained by asking CPLEX what
        the actual size is, rather than using sizes from when the problem
        was built.  cur_numrows and cur_numcols store the current number 
        of rows and columns, respectively.  */

        cur_numrows = CPXgetnumrows (env, lp);
        cur_numcols = CPXgetnumcols (env, lp);

        x = (double *) malloc (cur_numcols * sizeof(double));
        slack = (double *) malloc (cur_numrows * sizeof(double));
        dj = (double *) malloc (cur_numcols * sizeof(double));
        pi = (double *) malloc (cur_numrows * sizeof(double));

        if ( x     == NULL ||
        slack == NULL ||
        dj    == NULL ||
        pi    == NULL   ) {
        status = CPXERR_NO_MEMORY;
        fprintf (stderr, "Could not allocate memory for solution.\n");
        goto TERMINATE;
        }

        status = CPXsolution (env, lp, &solstat, &objval, x, pi, slack, dj);
        if ( status ) {
        exitflag = -1;
        fprintf (stderr, "Failed to obtain solution.\n");
        goto TERMINATE;
        }

//        /* Write the output to the screen. */

//        printf ("\nSolution status = %d\n", solstat);
//        printf ("Solution value  = %f\n\n", objval);

//        for (i = 0; i < cur_numrows; i++) {
//        printf ("Row %d:  Slack = %10f  Pi = %10f\n", i, slack[i], pi[i]);
//        }

//        for (j = 0; j < cur_numcols; j++) {
//        printf ("Column %d:  Value = %10f  Reduced cost = %10f\n",
//              j, x[j], dj[j]);
//        }

//        /* Finally, write a copy of the problem to a file. */

//        status = CPXwriteprob (env, lp, "lpex1.lp", NULL);
//        if ( status ) {
//        fprintf (stderr, "Failed to write LP to disk.\n");
//        goto TERMINATE;
//        }


        // Get objective value
        objvaluemin(0,0) = objval;

       
        // Maximization
        
        // Setup problem
        
        status = CPXchgobjsen (env, lp, CPX_MAX);  /* Problem is maximization */
        if ( status ){
        fprintf (stderr, "Failed to populate problem in: problem setup (max).\n");
        goto TERMINATE;
        }          
        
        
        /* Optimize the problem and obtain solution. */

        status = CPXlpopt (env, lp);
        if ( status ) {
        fprintf (stderr, "Failed to optimize LP.\n");
        exitflag = -1;
        goto TERMINATE;
        }

        /* The size of the problem should be obtained by asking CPLEX what
        the actual size is, rather than using sizes from when the problem
        was built.  cur_numrows and cur_numcols store the current number 
        of rows and columns, respectively.  */

//        cur_numrows = CPXgetnumrows (env, lp);
//        cur_numcols = CPXgetnumcols (env, lp);

//        x = (double *) malloc (cur_numcols * sizeof(double));
//        slack = (double *) malloc (cur_numrows * sizeof(double));
//        dj = (double *) malloc (cur_numcols * sizeof(double));
//        pi = (double *) malloc (cur_numrows * sizeof(double));

//        if ( x     == NULL ||
//        slack == NULL ||
//        dj    == NULL ||
//        pi    == NULL   ) {
//        status = CPXERR_NO_MEMORY;
//        fprintf (stderr, "Could not allocate memory for solution.\n");
//        goto TERMINATE;
//        }

        status = CPXsolution (env, lp, &solstat, &objval, x, pi, slack, dj);
        if ( status ) {
        exitflag = -1;
        fprintf (stderr, "Failed to obtain solution.\n");
        goto TERMINATE;
        }      
        
        
        // Get objective value
        objvaluemax(0,0) = objval;      
        
        
//        std::cout << "rhs ";
//        for(j=0; j<totalrows; j++)
//        {
//                std::cout << rhs[j] << " ";
//        }
//        std::cout << std::endl;   

        
        // Solve remaining LPs
        for(int j=1; j<nof_LPs; j++)
        {
                // Change LP
                
                obj = f.col(j).data(); 
                status = CPXchgobj(env, lp, nof_x, objind, obj);  /* Problem is minimization */
                if ( status ){
                fprintf (stderr, "Failed to change objective function.\n");
                goto TERMINATE;
                }                   
                
                // Minimization
                
                // Setup problem
                
                status = CPXchgobjsen (env, lp, CPX_MIN);  /* Problem is minimization */
                if ( status ){
                fprintf (stderr, "Failed to populate problem in: problem setup (min).\n");
                goto TERMINATE;
                }      
                
                /* Optimize the problem and obtain solution. */

                status = CPXlpopt (env, lp);
                if ( status ) {
                fprintf (stderr, "Failed to optimize LP.\n");
                exitflag = -1;
                goto TERMINATE;
                }         
                
                
                status = CPXsolution (env, lp, &solstat, &objval, x, pi, slack, dj);
                if ( status ) {
                exitflag = -1;
                fprintf (stderr, "Failed to obtain solution.\n");
                goto TERMINATE;
                }      


                // Get objective value                
                objvaluemin(j,0) = objval;
              
                
                // Maximization
                
                // Setup problem
                
                status = CPXchgobjsen (env, lp, CPX_MAX);  /* Problem is maximization */
                if ( status ){
                fprintf (stderr, "Failed to populate problem in: problem setup (max).\n");
                goto TERMINATE;
                }          
                
                
                /* Optimize the problem and obtain solution. */

                status = CPXlpopt (env, lp);
                if ( status ) {
                fprintf (stderr, "Failed to optimize LP.\n");
                exitflag = -1;
                goto TERMINATE;
                }      
                
                status = CPXsolution (env, lp, &solstat, &objval, x, pi, slack, dj);
                if ( status ) {
                exitflag = -1;
                fprintf (stderr, "Failed to obtain solution.\n");
                goto TERMINATE;
                }      
                
                // Get objective value                  
                objvaluemax(j,0) = objval;                                            
                        
        }      

        TERMINATE:

        /* Free up the solution */

        czonotope::free_and_null ((char **) &x);
        czonotope::free_and_null ((char **) &slack);
        czonotope::free_and_null ((char **) &dj);
        czonotope::free_and_null ((char **) &pi);

        /* Free up the problem as allocated by CPXcreateprob, if necessary */

        if ( lp != NULL ) {
        status = CPXfreeprob (env, &lp);
        if ( status ) {
         fprintf (stderr, "CPXfreeprob failed, error code %d.\n", status);
        }
        }

        /* Free up the CPLEX environment, if necessary */

        if ( env != NULL ) {
        status = CPXcloseCPLEX (&env);

        /* Note that CPXcloseCPLEX produces no output,
         so the only way to see the cause of the error is to use
         CPXgeterrorstring.  For other CPLEX routines, the errors will
         be seen if the CPXPARAM_ScreenOutput indicator is set to CPX_ON. */

        if ( status ) {
         char  errmsg[CPXMESSAGEBUFSIZE];
         fprintf (stderr, "Could not close CPLEX environment.\n");
         CPXgeterrorstring (env, status, errmsg);
         fprintf (stderr, "%s", errmsg);
        }
        }
                     
        
}

void czonotope::cplexqpwrapper(Eigen::MatrixXd H, Eigen::MatrixXd f, Eigen::MatrixXd A, Eigen::MatrixXd b, Eigen::MatrixXd Aeq, Eigen::MatrixXd beq, Eigen::MatrixXd LB, Eigen::MatrixXd UB, double& objvalue, Eigen::MatrixXd& xstar, int& exitflag)
{

        // CPLEX Quadratic Programming Wrapper: Callable Libraries version

        // Calls CPLEX to solve min 0.5*x'Hx + f'x subject to
        //
        //                      A*x <= b
        //                      Aeq*x = beq
        //                      LB <= x <= UB

        // H, f, A, b, Aeq, beq, LB and UB are MatrixXd objects of appropriate dimensions

        // Returns the optimal value of the objective function, the optimal value of x, and the exit flag from cplex
        
        
        exitflag = 0;
        
        int nof_x = f.rows();
        int nof_ineq = b.rows();
        int nof_eq = beq.rows();
        int nof_bounds = UB.rows();
        
        double* obj = f.data();
        double lb[nof_x];
        double ub[nof_x];    
        
        int i, j;        
        
        Eigen::MatrixXd Atotal(nof_ineq+nof_eq, nof_x);
        Eigen::MatrixXd btotal(nof_ineq+nof_eq, 1);
        if(nof_ineq!=0 && nof_eq==0)
        {
                Atotal = A;
                btotal = b;
        }
        else if(nof_ineq==0 && nof_eq!=0)
        {
                Atotal = Aeq;
                btotal = beq;
        }
        else
        {
                Atotal << A, Aeq;
                btotal << b, beq;
        }     
        
//        std::cout << "btotal " << btotal << std::endl;
        
        int totalrows = nof_ineq+nof_eq;
        
//        std::cout << "totalrows " << totalrows << std::endl;
        
        // Bounds on decision variables
        
//        double* obj = f.data();
//        double lb[nof_x];
//        double ub[nof_x];

        
        // Allocating arrays (one nested for loop)
        
        int      matbeg[nof_x];
        int      matind[nof_x*totalrows];        
        double*  matval = Atotal.data();
        double*  rhs = btotal.data();
        char     sense[totalrows];   
        int      qmatbeg[nof_x];
        int      qmatcnt[nof_x];
        int      qmatind[nof_x*nof_x];
        double   *qmatval = H.data();         
        
        if(nof_bounds==0)
        {
                for(j = 0; j<nof_x; j++)
                {
                        lb[j] = -CPX_INFBOUND;
                        ub[j] =  CPX_INFBOUND;
                        
                        matbeg[j] = j*totalrows;
                        
                        qmatbeg[j] = j*nof_x;
                        qmatcnt[j] = nof_x;
                        
                        for(i=0; i<nof_ineq; i++)
                        {
                                sense[i] = 'L';
                                matind[j*totalrows + i] = i;                      
                        }
                        for(i=nof_ineq; i<totalrows; i++)
                        {
                                sense[i] = 'E';
                                matind[j*totalrows + i] = i;                      
                        }   
                         
                        for(i=0; i<nof_x; i++)
                        {
                                qmatind[j*nof_x + i] = i;
                        }                          
                                          
                }
        }        
        else
        {
                
                for(j = 0; j<nof_x; j++)
                {
                        lb[j] =  LB(j,0);
                        ub[j] =  UB(j,0);

                        matbeg[j] = j*totalrows;
                        
                        qmatbeg[j] = j*nof_x;
                        qmatcnt[j] = nof_x;
                        
                        for(i=0; i<nof_ineq; i++)
                        {
                                sense[i] = 'L';
                                matind[j*totalrows + i] = i;                      
                        }
                        for(i=nof_ineq; i<totalrows; i++)
                        {
                                sense[i] = 'E';
                                matind[j*totalrows + i] = i;                      
                        }   
                         
                        for(i=0; i<nof_x; i++)
                        {
                                qmatind[j*nof_x + i] = i;
                        }                             
                }
        }                 
        

	// Code based on lpex1.c and qpex1.c, populatebyrows and populatebycols
	
	int      solstat;
        double   objval;
        double   *x = NULL;
        double   *pi = NULL;
        double   *slack = NULL;
        double   *dj = NULL;
        
        CPXENVptr     env = NULL;
        CPXLPptr      lp = NULL;
        int           status = 0;
        int           cur_numrows, cur_numcols;    
        
        /* Initialize the CPLEX environment */

        env = CPXopenCPLEX (&status);

        /* If an error occurs, the status value indicates the reason for
        failure.  A call to CPXgeterrorstring will produce the text of
        the error message.  Note that CPXopenCPLEX produces no output,
        so the only way to see the cause of the error is to use
        CPXgeterrorstring.  For other CPLEX routines, the errors will
        be seen if the CPXPARAM_ScreenOutput indicator is set to CPX_ON.  */

        if ( env == NULL ) {
        char  errmsg[CPXMESSAGEBUFSIZE];
        fprintf (stderr, "Could not open CPLEX environment.\n");
        CPXgeterrorstring (env, status, errmsg);
        fprintf (stderr, "%s", errmsg);
        goto TERMINATE;
        }
        
        /* Turn on output to the screen */

        status = CPXsetintparam (env, CPXPARAM_ScreenOutput, CPX_ON);
        //status = CPXsetintparam (env, CPXPARAM_ScreenOutput, CPX_OFF);
        if ( status ) {
        fprintf (stderr, 
               "Failure to turn on screen indicator, error %d.\n", status);
        goto TERMINATE;
        }
        status = CPXsetintparam (env, CPX_PARAM_SCRIND, CPX_ON);
        //status = CPXsetintparam (env, CPX_PARAM_SCRIND, CPX_OFF);
        if ( status ) {
        fprintf (stderr, 
               "Failure to turn on screen indicator, error %d.\n", status);
        goto TERMINATE;
        }        
        

        /* Turn on data checking */

        status = CPXsetintparam (env, CPXPARAM_Read_DataCheck,
	                    CPX_DATACHECK_WARN);
        if ( status ) {
        fprintf (stderr, 
               "Failure to turn on data checking, error %d.\n", status);
        goto TERMINATE;
        }   
        
        /* Create the problem. */

        lp = CPXcreateprob (env, &status, "lpex1");

        /* A returned pointer of NULL may mean that not enough memory
        was available or there was some other problem.  In the case of 
        failure, an error message will have been written to the error 
        channel from inside CPLEX.  In this example, the setting of
        the parameter CPXPARAM_ScreenOutput causes the error message to
        appear on stdout.  */

        if ( lp == NULL ) {
        fprintf (stderr, "Failed to create LP.\n");
        goto TERMINATE;
        }             
        
            
        
        
        // Setup problem
        
        //status = CPXchgobjsen (env, lp, CPX_MAX);  /* Problem is maximization */
        status = CPXchgobjsen (env, lp, CPX_MIN);  /* Problem is minimization */
        if ( status ){
        fprintf (stderr, "Failed to populate problem in: problem setup.\n");
        goto TERMINATE;
        }     
        
        
        // Create rows     
        
        status = CPXnewrows (env, lp, totalrows, rhs, sense, NULL, NULL);
        if ( status ){
        fprintf (stderr, "Failed to populate problem in: create rows.\n");
        goto TERMINATE;
        }        
        
        // Add cols
        
        status = CPXaddcols (env, lp, nof_x, nof_x*totalrows, obj, matbeg, matind,
                                matval, lb, ub, NULL);
        if ( status ){
        fprintf (stderr, "Failed to populate problem in: add cols.\n");
        goto TERMINATE;
        }
        
        
        /* Now copy the QP part of the problem data into the lp */

        status = CPXcopyquad (env, lp, qmatbeg, qmatcnt, qmatind, qmatval);
        if ( status ) {
        fprintf (stderr, "Failed to copy quadratic matrix.\n");
        goto TERMINATE;
        }        
        

        /* Optimize the problem and obtain solution. */

        status = CPXqpopt (env, lp);
        if ( status ) {
        fprintf (stderr, "Failed to optimize QP.\n");
        exitflag = -1;
        goto TERMINATE;
        }

        /* The size of the problem should be obtained by asking CPLEX what
        the actual size is, rather than using sizes from when the problem
        was built.  cur_numrows and cur_numcols store the current number 
        of rows and columns, respectively.  */

        cur_numrows = CPXgetnumrows (env, lp);
        cur_numcols = CPXgetnumcols (env, lp);

        x = (double *) malloc (cur_numcols * sizeof(double));
        slack = (double *) malloc (cur_numrows * sizeof(double));
        dj = (double *) malloc (cur_numcols * sizeof(double));
        pi = (double *) malloc (cur_numrows * sizeof(double));

        if ( x     == NULL ||
        slack == NULL ||
        dj    == NULL ||
        pi    == NULL   ) {
        status = CPXERR_NO_MEMORY;
        fprintf (stderr, "Could not allocate memory for solution.\n");
        goto TERMINATE;
        }

        status = CPXsolution (env, lp, &solstat, &objval, x, pi, slack, dj);
        if ( status ) {
        exitflag = -1;
        fprintf (stderr, "Failed to obtain solution.\n");
        goto TERMINATE;
        }

//        /* Write the output to the screen. */

//        printf ("\nSolution status = %d\n", solstat);
//        printf ("Solution value  = %f\n\n", objval);

//        for (i = 0; i < cur_numrows; i++) {
//        printf ("Row %d:  Slack = %10f  Pi = %10f\n", i, slack[i], pi[i]);
//        }

//        for (j = 0; j < cur_numcols; j++) {
//        printf ("Column %d:  Value = %10f  Reduced cost = %10f\n",
//              j, x[j], dj[j]);
//        }

//        /* Finally, write a copy of the problem to a file. */

//        status = CPXwriteprob (env, lp, "lpex1.lp", NULL);
//        if ( status ) {
//        fprintf (stderr, "Failed to write LP to disk.\n");
//        goto TERMINATE;
//        }

        objvalue = objval;
        for(int j=0; j<nof_x; j++)
        {
                xstar(j,0) = x[j]; // Optimal solution
        }
        
//        std::cout << "rhs ";
//        for(j=0; j<totalrows; j++)
//        {
//                std::cout << rhs[j] << " ";
//        }
//        std::cout << std::endl;         

        TERMINATE:

        /* Free up the solution */

        free_and_null ((char **) &x);
        free_and_null ((char **) &slack);
        free_and_null ((char **) &dj);
        free_and_null ((char **) &pi);

        /* Free up the problem as allocated by CPXcreateprob, if necessary */

        if ( lp != NULL ) {
        status = CPXfreeprob (env, &lp);
        if ( status ) {
         fprintf (stderr, "CPXfreeprob failed, error code %d.\n", status);
        }
        }

        /* Free up the CPLEX environment, if necessary */

        if ( env != NULL ) {
        status = CPXcloseCPLEX (&env);

        /* Note that CPXcloseCPLEX produces no output,
         so the only way to see the cause of the error is to use
         CPXgeterrorstring.  For other CPLEX routines, the errors will
         be seen if the CPXPARAM_ScreenOutput indicator is set to CPX_ON. */

        if ( status ) {
         char  errmsg[CPXMESSAGEBUFSIZE];
         fprintf (stderr, "Could not close CPLEX environment.\n");
         CPXgeterrorstring (env, status, errmsg);
         fprintf (stderr, "%s", errmsg);
        }
        }
        
        
}

void czonotope::luwrapper(Eigen::MatrixXd A, Eigen::MatrixXd& L, Eigen::MatrixXd& U, Eigen::PermutationMatrix<Eigen::Dynamic,Eigen::Dynamic>& P)
{
        /* Wrapper for Eigen's Partial Pivoting LU decomposition.
           Returns matrices L and U according to MATLAB syntax [L,U,p] = lu(A,'vector')
           Returns also the permutation matrix P as a PermutationMatrix object */
           
           
        Eigen::PartialPivLU<Eigen::MatrixXd> lu = Eigen::PartialPivLU<Eigen::MatrixXd>(A);
        
        L = lu.matrixLU().triangularView<Eigen::UnitLower>();
        U = lu.matrixLU().triangularView<Eigen::Upper>(); 
        P = lu.permutationP();  
 //       Eigen::PermutationMatrix<Eigen::Dynamic,Eigen::Dynamic> perm(2);
//        perm = lu.permutationP();        
        
}

void czonotope::free_and_null (char **ptr)
{
   if ( *ptr != NULL ) {
      free (*ptr);
      *ptr = NULL;
   }
} /* END free_and_null */ 



//};	
