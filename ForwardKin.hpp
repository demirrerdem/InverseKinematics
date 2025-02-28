#pragma once
#include "Eigen"
#include "DH.hpp"
#include <iostream>


class FK : public DH
{
public:
	FK() {};
	FK(Eigen::VectorXd _alpha, Eigen::VectorXd _a, Eigen::VectorXd _d, Eigen::VectorXd _tetha);


//protected:
	
	int sizeOfLink;
	std::vector<Eigen::Matrix4d>  TL;  // link to link Transformation matrix
	std::vector<Eigen::Matrix4d>  T0N; // base to link Transformation matrix
	Eigen::Vector3d endEffector;
	void TransF();

	void setTetha(Eigen::VectorXd _tetha);



};




