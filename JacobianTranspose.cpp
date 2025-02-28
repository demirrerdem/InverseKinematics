#include "JacobianTranspose.hpp"

JacobianTranspose::JacobianTranspose(Eigen::VectorXd _alpha, Eigen::VectorXd _a,
	Eigen::VectorXd _d, Eigen::VectorXd _tetha, int _iter, double _tolerance)
	: FK(_alpha, _a, _d, _tetha), Solver(_alpha.size(), _iter, _tolerance)
{

}

void JacobianTranspose::Calculate(Eigen::Vector3d _target)
{
	target = _target;

	for (Iter = 0; Iter < maxIter; Iter++)
	{
		// positon of end effector
		Eigen::Vector3d EE = getPos(T0N.back());

		// error vector between target - end effector
		Eigen::Vector3d posError = target - EE;

		// norm of error vector
		Error = posError.norm();


		if (Error < tolerance)
		{
			return;
		}

		// calculate jacobian matrix
		calcJacobian(T0N, sizeLink);


		Eigen::VectorXd deltaQ(sizeLink);

		// finding new delta tetha 
		deltaQ = Jacobian.transpose() * posError;


		
		double num = posError.transpose() * (Jacobian * Jacobian.transpose())* posError;

		Eigen::Vector3d temp = Jacobian * deltaQ;

		double denom = std::pow(temp.norm(),2);


		float beta_;

		if (denom > 1e-6)
			beta_ = num / denom;
		else
			beta_ = beta;
		
		// find new tetha
		tetha = tetha + deltaQ * beta_;

		// set new tetha
		setTetha(tetha);

		// update transformation matrix
		TransF();

		// debug
		//std::cout << "tetha:  " << tetha << "\n";

	}

}

void JacobianTranspose::setBeta(float _beta)
{
	beta = _beta;
}
