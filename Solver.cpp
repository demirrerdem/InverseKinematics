#include "Solver.hpp"


Solver::Solver(int s, int _iter, double _tolerance)
	: sizeLink{ s }, maxIter{ _iter }, tolerance{ _tolerance }
{

}

int Solver::getIter()
{
	return Iter;
}

double Solver::getError()
{
	return Error;
}

void Solver::calcJacobian(std::vector<Eigen::Matrix4d>  _T0N, int _sizeLink)
{
	for (int i = 0; i < _sizeLink; i++)
	{
		// get z rotation axis form TF
		Eigen::Vector3d rotation = getRot(_T0N.at(i));

		// get position vector from TF
		Eigen::Vector3d position = getPos(_T0N.at(i));

		// calculate position; end efector to current link 
		Eigen::Vector3d EEtoLink = currLinktoEE(getPos(_T0N.back()), getPos(_T0N.at(i)));
		
		// cross pruduct between rotation vector and EEtoLink vector
		Eigen::Vector3d jako = rotation.cross(EEtoLink);

		// update Jacobian matrix
		Jacobian.col(i) = jako;
	}

	// std::cout << Jacobian;
}

Eigen::Vector3d Solver::currLinktoEE(Eigen::Vector3d endEffector, Eigen::Vector3d currLink)
{
	// calculate position; end efector to current link 
	return endEffector - currLink;
}

Eigen::Vector3d Solver::calcError(Eigen::Vector3d target, Eigen::Vector3d endEffector)
{
	// calculate error
	return target - endEffector;
}
