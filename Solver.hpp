#pragma once
#include "Eigen"
#include "Tools.hpp"
#include <iostream>

class Solver
{

public:
	Solver() {};
	Solver(int s, int _iter, double _tolerance);
	virtual ~Solver() {};

	int		getIter();
	double	getError();

	// pure virtual Calulate error, all methot have to use this func for IK
	virtual void Calculate(Eigen::Vector3d _target) = 0;

protected:
	
	int sizeLink;

	int maxIter;

	double tolerance;

	int Iter;

	double Error;

	// Target Point vectors
	Eigen::Vector3d target;

	// matrix of jacobian (initialize with 0)
	Eigen::MatrixXd  Jacobian = Eigen::MatrixXd::Zero(3, sizeLink);

	// calculate jacobian
	void calcJacobian(std::vector<Eigen::Matrix4d>  _T0N, int _sizeLink);

	// End effector pos - current Link (for calculating Jacobian)
	Eigen::Vector3d currLinktoEE(Eigen::Vector3d endEffector, Eigen::Vector3d currLink);

	// calculate error
	Eigen::Vector3d calcError(Eigen::Vector3d target, Eigen::Vector3d endEffector);



};