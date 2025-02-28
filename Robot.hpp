#pragma once
#include <iostream>
#include <SFML/Graphics.hpp>
#include "Eigen"
#include "Tools.hpp"

class Robot
{

public:

	Robot() {};
	
	// Robot draw function
	void Draw(sf::RenderWindow* window, std::vector<Eigen::Matrix4d> Tf, Eigen::Vector3d target);

	// change link size
	void setSize(float width);

	void setOffset(float x, float y);

private:

	float linkWidth = 10;

	float getAngle(Eigen::Vector3d point1, Eigen::Vector3d point2);
	float getLength(Eigen::Vector3d point1, Eigen::Vector3d point2);

	float yoffset{ 300 };
	float xoffset{ 400 };

};