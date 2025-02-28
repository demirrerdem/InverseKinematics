#pragma once

#include "Solver.hpp"
#include "ForwardKin.hpp"
#include "Tools.hpp"



class NewtonRaphson :  public Solver, public FK
{

public:

	NewtonRaphson() {};
	NewtonRaphson(Eigen::VectorXd _alpha, Eigen::VectorXd _a, Eigen::VectorXd _d , 
					Eigen::VectorXd _tetha, int _iter,double _tolerance);

	// NewtonRamphson methot
	void Calculate(Eigen::Vector3d _target) override;

};
