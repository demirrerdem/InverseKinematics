#pragma once

#include "Solver.hpp"
#include "ForwardKin.hpp"
#include "Tools.hpp"


class JacobianTranspose : public Solver, public FK
{
public:
	JacobianTranspose() {};
	JacobianTranspose(Eigen::VectorXd _alpha, Eigen::VectorXd _a, Eigen::VectorXd _d,
		Eigen::VectorXd _tetha, int _iter, double _tolerance);


	void Calculate(Eigen::Vector3d _target) override;

	void setBeta(float _beta);

private:

	float beta{ 0.01 };

};

