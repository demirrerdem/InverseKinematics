#include "Robot.hpp"

void Robot::Draw(sf::RenderWindow* window, std::vector<Eigen::Matrix4d> Tf, Eigen::Vector3d target)
{
    int sizeLink = Tf.size();
	for (int i = 1; i < sizeLink; i++)
	{
        sf::RectangleShape link;
        
        sf::CircleShape targetPoint(10);

        sf::CircleShape joint(10);



        joint.setFillColor(sf::Color{ 120, 99, 212 });
        joint.setOutlineThickness(1);
        joint.setOutlineColor(sf::Color::Black);
        joint.setOrigin(sf::Vector2f(10, 10));

        targetPoint.setFillColor(sf::Color::Green);
        targetPoint.setOrigin(sf::Vector2f(10, 10));
        targetPoint.setPosition(target(0) + xoffset, target(1) + yoffset);

		link.setFillColor(sf::Color{ 224, 208, 126 });

        Eigen::Vector3d point1 = getPos(Tf.at(i - 1));
        Eigen::Vector3d point2 = getPos(Tf.at(i));

        if (i == 1)
        {
            joint.setFillColor(sf::Color{ 245, 122, 137 });
        }

        joint.setPosition(sf::Vector2f(point1(0) + xoffset, point1(1) + yoffset));

        // calculate angle between two points
        float angle = getAngle(point1, point2);

        // calculate length between two points
        float length = getLength(point1, point2);

        // setting of line
        link.setSize(sf::Vector2f(length, linkWidth));
        link.setOrigin(sf::Vector2f(0, linkWidth / 2));


        // set position
        link.setPosition(point1(0) + xoffset, point1(1) + yoffset);

        // set roatation 
        link.setRotation(angle);

        // draw link

        window->draw(targetPoint);
        window->draw(link);
        window->draw(joint);

	}


}

void Robot::setSize(float width)
{
	linkWidth	= width;
}

void Robot::setOffset(float x, float y)
{
    xoffset = x;
    yoffset = y;
}



float Robot::getAngle(Eigen::Vector3d point1, Eigen::Vector3d point2)
{
    return std::atan2(point2(1) - point1(1), point2(0) - point1(0)) * 180 / 3.14159f;
}

float Robot::getLength(Eigen::Vector3d point1, Eigen::Vector3d point2)
{
    return std::sqrt(std::pow(point2(0) - point1(0), 2) + std::pow(point2(1) - point1(1), 2));
}

