#include "Tools.hpp"

Eigen::Vector3d getPos(Eigen::Matrix4d Tf)
{
    // get position from transdormation matrix
    Eigen::Vector3d pos;
    pos(0) = Tf(0, 3);
    pos(1) = Tf(1, 3);
    pos(2) = Tf(2, 3);

    return pos;
}

Eigen::Vector3d getRot(Eigen::Matrix4d Tf)
{
    // get rotation matrix's z axis
    Eigen::Vector3d rot;
    rot(0) = Tf(0, 2);
    rot(1) = Tf(1, 2);
    rot(2) = Tf(2, 2);

    return rot;
}