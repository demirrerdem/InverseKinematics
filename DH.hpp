#pragma once
#include "Eigen"

class DH
{
protected:

	// D-H variables
	Eigen::VectorXd alpha;
	Eigen::VectorXd a;
	Eigen::VectorXd d;
	Eigen::VectorXd tetha;

public:
	DH(Eigen::VectorXd _alpha, Eigen::VectorXd _a, Eigen::VectorXd _d, Eigen::VectorXd _tetha);
	DH();

};
