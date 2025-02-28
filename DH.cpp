#include "DH.hpp"
#include <iostream>

#define PI   3.1415926535

DH::DH()
{
	alpha	= Eigen::Vector4d::Zero();
	a		= Eigen::Vector4d::Zero();
	d		= Eigen::Vector4d::Zero();
	tetha	= Eigen::Vector4d::Zero();
}

DH::DH(Eigen::VectorXd _alpha, Eigen::VectorXd _a, Eigen::VectorXd _d, Eigen::VectorXd _tetha)

{
	alpha	= _alpha * PI / 180; // Degree to radian
	a		= _a;
	d		= _d;
	tetha	= _tetha;
}
