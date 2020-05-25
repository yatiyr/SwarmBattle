/*
 * Engine.cpp
 *
 *  Created on: Mar 24, 2020
 *      Author: Eren
 */

#include "Engine.h"
#include "SensorContactListener.h"

SensorContactListener listener;

// Config is going to be read
Engine::Engine() {

	readMainSettings();
	window = new sf::RenderWindow(sf::VideoMode(wwidth, wheight), "Simulation");
	sf::View view(sf::FloatRect(0.f,0.f,wwidth,wheight));
	world = new b2World(gravity);
	world->SetContactListener(&listener);
	objectFactory = new ObjectFactory(world,window,scale,wwidth,wheight);
	window->setView(view);

}



Engine::~Engine() {}

void Engine::readMainSettings() {
	std::ifstream file("mainSettings.config");
	std::string line;
	std::string delimeter = "\t";
	std::string token;
	std::string value;

	// Extract TimeStep
	std::getline(file,line);
	token  = line.substr(line.find(delimeter),line.size()-1);
	timeStep = 1/std::stof(token);

	// Extract VelocityIt and PositionIt
	std::getline(file,line);
	token = line.substr(line.find(delimeter),line.size()-1);
	velocityIterations = std::stoi(token);

	std::getline(file,line);
	token = line.substr(line.find(delimeter),line.size()-1);
	positionIterations = std::stoi(token);

	// Get WindowColor
	std::getline(file,line);
	token = line.substr(line.find(delimeter),line.size()-1);
	std::vector<std::string> valuesWindowColor;
	const char delimWindowColor = ',';
	split(token, delimWindowColor, valuesWindowColor);
	int red = std::stoi(valuesWindowColor[0]);
	int green = std::stoi(valuesWindowColor[1]);
	int blue = std::stoi(valuesWindowColor[2]);
	int alpha = std::stoi(valuesWindowColor[3]);
	windowColor.r = red;
	windowColor.g = green;
	windowColor.b = blue;
	windowColor.a = alpha;

	// Get WWidth
	std::getline(file,line);
	token = line.substr(line.find(delimeter),line.size()-1);
	wwidth = std::stoi(token);

	// Get WHeight
	std::getline(file,line);
	token = line.substr(line.find(delimeter), line.size()-1);
	wheight = std::stoi(token);

	// Get Gravity
	std::getline(file,line);
	token = line.substr(line.find(delimeter),line.size()-1);
	std::vector<std::string> valuesGravity;
	const char delimGravity = ',';
	split(token, delimGravity, valuesGravity);
	gravity.x = std::stof(valuesGravity[0]);
	gravity.y = std::stof(valuesGravity[1]);

	// Get Scale
	std::getline(file,line);
	token = line.substr(line.find(delimeter),line.size()-1);
	scale = std::stoi(token);

}

void Engine::addObject(Object *obj) {

	objects.push_back(obj);

}

void Engine::actObjects() {

	for(unsigned int i=0;i<objects.size();i++) {
		objects[i]->act();
	}
}

void Engine::drawObjects() {

	for(unsigned int i=0;i<objects.size();i++) {
		objects[i]->draw(scale,wwidth,wheight);
	}
}

void Engine::addRobot(Robot *r) {
	objects.push_back(r);
	robots.push_back(r);
}

void Engine::addBase(Base *b) {
	objects.push_back(b);
	bases.push_back(b);
}

void Engine::actRobots() {

	for(unsigned int i=0;i<robots.size();i++) {
		robots[i]->act();
	}
}

void Engine::drawRobots() {

	for(unsigned int i=0;i<robots.size();i++) {
		robots[i]->draw(scale,wwidth,wheight);
	}
}

void Engine::drawBases() {
	for(unsigned int i=0;i<bases.size();i++) {
		bases[i]->draw(scale,wwidth,wheight);
	}
}

void Engine::addRocket(Rocket *r) {
	objects.push_back(r);
	rockets.push_back(r);
}


void Engine::run() {

	  sf::Texture GroundTexture;

	  // Sprites
	  sf::Sprite GroundSprite;
	  GroundSprite.setTexture(GroundTexture);
	  GroundSprite.setColor(sf::Color(0, 0, 0, 255));
	  GroundSprite.setTextureRect(sf::IntRect(0, 0, 12000 * scale, 2 * scale));
	  GroundSprite.setOrigin(6000 * scale, 1 * scale); // origin in middle

//	  for(int i=0;i<100;i++) {
//		  std::cout << i << std::endl;
//		  Robot *r = objectFactory->createRobot(b2Vec2(100.f*i,0*i), sf::Color::Black,1);
//		  objects.push_back(r);
//		  robots.push_back(r);
//	  }


//	  Rocket *rocket1 = objectFactory->createRocket(b2Vec2(23.f,149.f), sf::Color::Red);

	  Robot *r = objectFactory->createRobot(b2Vec2(10.f,-20.f),sf::Color::Black,1);
	  objects.push_back(r);
	  robots.push_back(r);
	  Base *base = objectFactory->createBase(b2Vec2(0.f,-185.f), sf::Color::Black, 1);
	  objects.push_back(base);
	  bases.push_back(base);

	  // Define the ground body.
	  b2BodyDef groundBodyDef;
	  groundBodyDef.position.Set(0.0f, -200.0f);


	  b2Body* groundBody = world->CreateBody(&groundBodyDef);
	  b2PolygonShape groundBox;
	  groundBox.SetAsBox(6000.f, 1.0f);
	  groundBody->CreateFixture(&groundBox, 0.0f);


	  window->setFramerateLimit(60);

	  while(window->isOpen()) {

		// Event handler
		sf::Event event;
		while(window->pollEvent(event)) {
			if(event.type == sf::Event::Closed) {
				window->close();
			}

			else if(event.type == sf::Event::MouseWheelMoved) {
				sf::View view = window->getView();
				if(event.mouseWheel.delta >= 1) {
					zoom = 0.95;
				}
				else if(event.mouseWheel.delta <= -1) {
					zoom = 1.05;
				}
				view.zoom(zoom);
				window->setView(view);
			}
			else if(event.type == sf::Event::Resized) {

				sf::FloatRect visibleArea(0.f,0.f,event.size.width, event.size.height);
				wwidth = event.size.width;
				wheight = event.size.height;
				window->setView(sf::View(visibleArea));
			}
			else if(event.type == sf::Event::MouseButtonPressed) {
				if(event.mouseButton.button == 0) {
					moving = true;
					oldPos = window->mapPixelToCoords(sf::Vector2i(event.mouseButton.x, event.mouseButton.y));
				}
			}
			else if(event.type == sf::Event::MouseButtonReleased) {

				if(event.mouseButton.button == 0) {
					moving = false;
				}
			}
			else if(event.type == sf::Event::MouseMoved) {
				sf::View view = window->getView();
				if(moving) {
					const sf::Vector2f newPos = window->mapPixelToCoords(sf::Vector2i(event.mouseMove.x, event.mouseMove.y));
					const sf::Vector2f deltaPos = oldPos-newPos;
					view.setCenter(view.getCenter() + deltaPos);
					window->setView(view);
                    oldPos = window->mapPixelToCoords(sf::Vector2i(event.mouseMove.x, event.mouseMove.y));
				}
			}
		}


		actRobots();
		window->clear(windowColor);
		world->Step(timeStep, velocityIterations, positionIterations);


	    // get the position and angle of the ground
	    b2Vec2 pos = groundBody->GetPosition();
	    float angle = groundBody->GetAngle();
	    pos = convertWorldToScreen( pos);
	    GroundSprite.setPosition(pos.x, pos.y);
	    GroundSprite.setRotation(-(180/b2_pi) * angle);
	    window->draw(GroundSprite);

	    drawRobots();
	    drawBases();

		window->display();

	}

}


