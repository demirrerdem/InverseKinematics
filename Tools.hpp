#pragma once
#include "Eigen"


Eigen::Vector3d getPos(Eigen::Matrix4d Tf);

Eigen::Vector3d getRot(Eigen::Matrix4d Tf);