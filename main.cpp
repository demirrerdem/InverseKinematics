#include <iostream>
#include <SFML/Graphics.hpp>
#include "Eigen"
#include "Robot.hpp"
#include "NewtonRaphson.hpp"
#include "JacobianTranspose.hpp"



int main()
{
    
	Eigen::VectorXd alpha(5);

	Eigen::VectorXd a(5);

	Eigen::VectorXd d(5);

	Eigen::VectorXd tetha(5);


	// DH table
	alpha	<< 0, 0, 0, 0, 0;
	a		<< 0, 70, 70, 70, 70;
	d		<< 0, 0, 0, 0, 0;
	tetha	<< 0, 0, 0, 0, 0;

	// tolerance
	double tol = 0.01;

	// max iter
	int iter = 1000;

	// target position
	Eigen::Vector3d target;
	target << 0, 0, 0;

   
	// set window size
	int xAxis = 800;
	int yAxis = 600;

    sf::RenderWindow window(sf::VideoMode(xAxis, yAxis), "IK");

	// -----------------------------

	sf::Text textIter;
	sf::Text textNR; // text of newthon ramphson
	sf::Text textJB; // text of jacobian based

	sf::Font font;
	if (!font.loadFromFile("Font/Arial.ttf"))
		std::cout << "Font Error";


	// select the font
	textIter.setFont(font);
	textNR.setFont(font);
	textJB.setFont(font);

	// set the character size
	textIter.setCharacterSize(15);
	textNR.setCharacterSize(15);
	textJB.setCharacterSize(15);

	// set the color
	textIter.setFillColor(sf::Color::Black);
	textNR.setFillColor(sf::Color::Blue);
	textJB.setFillColor(sf::Color::Black);

	// set the text style
	textIter.setStyle(sf::Text::Bold);
	textNR.setStyle(sf::Text::Bold);
	textJB.setStyle(sf::Text::Bold);

	textIter.setPosition(sf::Vector2f(xAxis-120,0));

	textNR.setString(" '1' : 'NewtonRaphson'");
	textJB.setString("\n '2' : 'JacobianTranspose'");


	// -----------------------------

	Robot* rbt = new Robot;

	// init method pointer
	Solver* method = new NewtonRaphson(alpha, a, d, tetha, iter, tol);

	std::vector<Eigen::Matrix4d>  _T0N;

	int numMethod = 1; // 1 : NewtonRaphson  2 : JacobianTranspose
    
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();

			if (event.type == sf::Event::KeyPressed) {
				if (event.key.code == sf::Keyboard::Num1) { // press num 1 

					// delete method and create new pointer
					delete method;
					method = new NewtonRaphson(alpha, a, d, tetha, iter, tol);

					// calculate with new method
					method->Calculate(target);
					numMethod = 1;
					

					// change text colors
					textNR.setFillColor(sf::Color::Blue);
					textJB.setFillColor(sf::Color::Black);

				}

				if (event.key.code == sf::Keyboard::Num2) { // press num 2 

					// delete method and create new pointer
					delete method;
					method = new JacobianTranspose(alpha, a, d, tetha, iter, tol);

					// calculate with new method
					method->Calculate(target);
					numMethod = 2;
					

					// change text colors
					textNR.setFillColor(sf::Color::Black);
					textJB.setFillColor(sf::Color::Blue);

				}
			}

			if (event.type == sf::Event::MouseButtonPressed) {
				if (event.mouseButton.button == sf::Mouse::Left) { // left click

					sf::Vector2i mousePos = sf::Mouse::getPosition(window);

					// refresh target position according to window coordinates
					target << mousePos.x - xAxis / 2, mousePos.y - yAxis / 2, 0;

					method->Calculate(target);

				}
			}
					
        }

		
		if		(numMethod == 1)
			_T0N = dynamic_cast<NewtonRaphson*>(method)->T0N;
		else if (numMethod == 2)
			_T0N = dynamic_cast<JacobianTranspose*>(method)->T0N;


		// --------------------------

		textIter.setString("Iteration: " + std::to_string(method->getIter()));

		// ---------------------------
		window.clear(sf::Color{ 109, 122, 143 });

		rbt->Draw(&window, _T0N,target);

		window.draw(textIter);
		window.draw(textNR);
		window.draw(textJB);

        window.display();    
    }

}
