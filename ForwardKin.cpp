#include "ForwardKin.hpp"

FK::FK(Eigen::VectorXd _alpha, Eigen::VectorXd _a, Eigen::VectorXd _d, Eigen::VectorXd _tetha) :
	DH(_alpha, _a, _d, _tetha)
{
	sizeOfLink = _alpha.size();
    TransF();
}

void FK::TransF()
{
    // clear all variable
    T0N.clear();
    TL.clear();

    for (int i = 0; i < sizeOfLink; i++)
    {
        // Equation of Transformation 
        Eigen::Matrix4d Tf{

            {std::cos(tetha[i]), - std::sin(tetha[i]), 0, a[i]},

            {std::sin(tetha[i]) * std::cos(alpha[i]), std::cos(tetha[i]) * std::cos(alpha[i]), - std::sin(alpha[i]), -std::sin(alpha[i]) * d[i] },

            {std::sin(tetha[i]) * sin(alpha[i]), std::cos(tetha[i]) * std::sin(alpha[i]), std::cos(alpha[i]), std::cos(alpha[i]) * d[i] },

            {0,0,0,1} };

        // update Tf for link to link
        TL.push_back(Tf);
    }

    // update Tf for base to link
    T0N.push_back(TL[0]);

    for (int i = 1; i < sizeOfLink; i++)
    {
        // calculate tf for base to link
        Eigen::Matrix4d newTf;

        newTf = T0N[i - 1] * TL[i];

        T0N.push_back(newTf);

    }

}

void FK::setTetha(Eigen::VectorXd _tetha)
{
    // to update theta vector
    tetha = _tetha;
}


