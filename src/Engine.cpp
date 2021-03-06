/*
 * Engine.cpp
 *
 *  Created on: Mar 24, 2020
 *      Author: Eren
 */

#include "Engine.h"
#include "SensorContactListener.h"
#include <math.h>
#include <algorithm>
#include <set>

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

void Engine::split(std::string const &str, const char delim,
		std::vector<std::string> &out) {
	size_t start;
	size_t end = 0;

	while((start = str.find_first_not_of(delim, end)) != std::string::npos) {
		end = str.find(delim, start);
		out.push_back(str.substr(start, end-start));
	}

}

b2Vec2 Engine::convertWorldToScreen( const b2Vec2& v) {
	return b2Vec2( (v.x * scale + wwidth/2) , -v.y * scale + wheight/4);
}

b2Vec2 Engine::convertScreenToWorld( const b2Vec2& v) {
	return b2Vec2( ((v.x - wwidth/2)/scale), -(v.y - wheight/4)/scale);
}

void Engine::addObject(Object *obj) {

	objects.push_back(obj);

}

void Engine::addRocket(Rocket *r) {
	rockets.push_back(r);
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

void Engine::drawRockets() {
	for(unsigned int i=0;i<rockets.size();i++) {
		rockets[i]->draw(scale,wwidth,wheight);
	}
}

void Engine::addRobot(Robot *r) {
	robots.push_back(r);
}

void Engine::addBase(Base *b) {
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

void Engine::drawParticles() {
	for(unsigned int i=0;i<particles.size();i++) {
		particles[i]->draw(scale,wwidth,wheight);
	}
}

void Engine::addParticle(Particle* p) {
	particles.push_back(p);
}

void Engine::handleRocketDrags() {
	for(unsigned int i=0;i<rockets.size();i++) {
		rockets[i]->handleDrag();
	}
}

// This method deletes dead entities
void Engine::clearDeadBodies() {

	std::set<Robot*> deadRobots;
	std::set<Rocket*> deadRockets;
	std::set<Particle*> deadParticles;
	std::set<Base*> deadBases;

	if(particles.size() >= 750) {
		std::vector<Particle*>::iterator itParticle = particles.begin();
		particles.erase(itParticle);
	}

	// Traverse robots
	for(unsigned int i=0;i<robots.size();i++) {

		if(robots[i]->getHp() <= 0) {

			for(int j=0;j<12;j++) {
				float angle = (j / (float)12) * 360 * DEGTORAD;
				b2Vec2 rayDir(sinf(angle), cosf(angle));

				Particle *p = objectFactory->createParticle(robots[i]->getBody()->GetPosition(),robots[i]->getColor(),robots[i]->getTeamId());
				addParticle(p);
				p->getBody()->SetLinearVelocity(100 * rayDir);
			}

			deadRobots.insert(robots[i]);
		}
	}

	for(unsigned int i=0;i<rockets.size();i++) {

		if(rockets[i]->getHp() <= 0) {
			for(int j=0;j<288;j++) {
				float angle = (j / (float)72) * 360 * DEGTORAD;
				b2Vec2 rayDir(sinf(angle), cosf(angle));

				Particle *p = objectFactory->createParticle(rockets[i]->getBody()->GetPosition(),rockets[i]->getColor(),rockets[i]->getTeamId());
				addParticle(p);
				p->getBody()->SetLinearVelocity(100 * rayDir);
			}

			deadRockets.insert(rockets[i]);
		}
	}

	for(unsigned int i=0;i<bases.size();i++) {

		if(bases[i]->getHp() <= 0) {

			for(int j=0;j<500;j++) {
				float angle = (j / (float)500) * 360 * DEGTORAD;
				b2Vec2 rayDir(sinf(angle), cosf(angle));

				Particle *p = objectFactory->createParticle(bases[i]->getBody()->GetPosition(),bases[i]->getColor(),bases[i]->getTeamId());
				addParticle(p);
				p->getBody()->SetLinearVelocity(100 * rayDir);
			}

			deadBases.insert(bases[i]);

		}
	}

	for(unsigned int i=0;i<particles.size();i++) {

		if(particles[i]->getHp() <= 0) {
			deadParticles.insert(particles[i]);
		}
	}

	std::set<Robot*>::iterator itRobot = deadRobots.begin();
	std::set<Robot*>::iterator endRobot = deadRobots.end();

	std::set<Rocket*>::iterator itRocket = deadRockets.begin();
	std::set<Rocket*>::iterator endRocket = deadRockets.end();

	std::set<Particle*>::iterator itParticle = deadParticles.begin();
	std::set<Particle*>::iterator endParticle = deadParticles.end();

	std::set<Base*>::iterator itBase = deadBases.begin();
	std::set<Base*>::iterator endBase = deadBases.end();


	// Clear dead bodies from lists and the world
	for(;itRobot!=endRobot; ++itRobot) {
		Robot* deadRobot = *itRobot;
		delete deadRobot;
		std::vector<Robot*>::iterator it = std::find(robots.begin(),robots.end(),deadRobot);
		if(it != robots.end()) {
			robots.erase(it);
		}
	}
	for(;itRocket!=endRocket; ++itRocket) {
		Rocket* deadRocket = *itRocket;
		delete deadRocket;
		std::vector<Rocket*>::iterator it = std::find(rockets.begin(),rockets.end(),deadRocket);
		if(it != rockets.end()) {
			rockets.erase(it);
		}
	}
	for(;itParticle!=endParticle; ++itParticle) {
		Particle* deadParticle = *itParticle;
		delete deadParticle;
		std::vector<Particle*>::iterator it = std::find(particles.begin(),particles.end(),deadParticle);
		if(it != particles.end()) {
			particles.erase(it);
		}
	}
	for(;itBase!=endBase; ++itBase) {
		Base* deadBase = *itBase;
		delete deadBase;
		std::vector<Base*>::iterator it = std::find(bases.begin(),bases.end(),deadBase);
		if(it != bases.end()) {
			bases.erase(it);
		}
	}
}

// Base fire up events are implemented in this
// function
void Engine::handleBase(Base *b) {
	BState gunState = b->getState();
	if(gunState == BState::GunSteady) {

	}
	else if(gunState == BState::Reloading) {

		b2Body *gunBody = b->getGunBody();
		b2Vec2 gunPos = gunBody->GetPosition();
		float gunAngle = gunBody->GetAngle();
		if((180/b2_pi)*gunAngle < 1.f && (180/b2_pi)*gunAngle > -1.f) {
			Rocket *rocket1;
			if(b->getTeamId() == 0) {
				rocket1 = objectFactory->createRocket(b2Vec2(gunPos.x,gunPos.y + 5.f),sf::Color::Blue,b->getTeamId());
			}
			else if(b->getTeamId() == 1) {
				rocket1 = objectFactory->createRocket(b2Vec2(gunPos.x,gunPos.y + 5.f),sf::Color::Red,b->getTeamId());
			}
			addRocket(rocket1);
			b->setReloadedRocket(rocket1);
			b->setState(BState::Adjusting);

			if(b->getTeamId() == 0) {
				srand(time(NULL));
				gunAng1 = rand()%55;
				gunAng1 += 15;
			}
			else if(b->getTeamId() == 1) {
				srand(time(NULL) + 10000);
				gunAng2 = rand()%55;
				gunAng2 += 15;
			}

		}
		else {
			if(gunAngle < 0) {
				gunBody->SetAngularVelocity(0.9f);
			}
			else if(gunAngle > 0) {
				gunBody->SetAngularVelocity(-0.9f);
			}
		}
	}
	else if(gunState == BState::GunFiring) {
		Rocket *reloadedRocket = b->getReloadedRocket();
		b2Body *rb = reloadedRocket->getBody();
		float rbAngle = rb->GetAngle();
		rb->SetEnabled(true);
		b2Vec2 force(-sin(rbAngle),cos(rbAngle));
		force *= 10000000;
		rb->ApplyLinearImpulseToCenter(force,true);
		b->setState(BState::Reloading);
	}
	else if(gunState == BState::Adjusting) {
		Rocket *reloadedRocket = b->getReloadedRocket();
		b2Body *rb = reloadedRocket->getBody();
		b2Body *gunBody = b->getGunBody();
		float gunAngle = gunBody->GetAngle();
		b2Vec2 gunPos = gunBody->GetPosition();
		b2Vec2 rPos = rb->GetPosition();
		b2Vec2 transformedPos = b2Vec2(gunPos.x - sin(gunAngle)*5,gunPos.y + cos(gunAngle)*5);
		reloadedRocket->getBody()->SetTransform(transformedPos,gunAngle);
		reloadedRocket->getBody()->SetEnabled(false);


		if(b->getTeamId() == 0) {
			if((180/b2_pi)*gunAngle >= -gunAng1 - 0.6f && (180/b2_pi)*gunAngle <= -gunAng1 + 0.6f ) {
				b->setState(BState::GunFiring);
			}
			else {
				gunBody->SetAngularVelocity(-0.6f);
			}
		}
		else if(b->getTeamId() == 1) {
			if((180/b2_pi)*gunAngle <= gunAng2 + 0.6f && (180/b2_pi)*gunAngle >= gunAng2 - 0.6f ) {
				b->setState(BState::GunFiring);
			}
			else {
				gunBody->SetAngularVelocity(0.6f);
			}
		}

	}
}

void Engine::handleBases() {
	for(unsigned int i=0;i<bases.size();i++) {
		handleBase(bases[i]);
	}
}

void Engine::run() {

	  sf::Texture GroundTexture;

	  // Sprites
	  sf::Sprite GroundSprite;
	  GroundSprite.setTexture(GroundTexture);
	  GroundSprite.setColor(sf::Color(0, 0, 0, 255));
	  GroundSprite.setTextureRect(sf::IntRect(0, 0, 12000 * scale, 2 * scale));
	  GroundSprite.setOrigin(6000 * scale, 1 * scale); // origin in middle



//	  Rocket *rocket1 = objectFactory->createRocket(b2Vec2(23.f,149.f), sf::Color::Red);

	  Base *base = objectFactory->createBase(b2Vec2(0.f,-185.f), sf::Color::Blue, 0);
	  bases.push_back(base);

	  Base *base2 = objectFactory->createBase(b2Vec2(1400.f,-185.f), sf::Color::Red,1);
	  bases.push_back(base2);

	  for(int k = 0;k<60;k++) {
		  Robot *r = objectFactory->createRobot(b2Vec2(k*(-3.f),100.f),sf::Color::Blue,0,base->getBody()->GetPosition(), timeStep);
//		  Robot *r2 = objectFactory->createRobot(b2Vec2((k*3.f)+1400,100.f),sf::Color::Red,0,base2->getBody()->GetPosition(), timeStep);
		  robots.push_back(r);
//		  robots.push_back(r2);
	  }



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

		clearDeadBodies();
		handleBases();
		handleRocketDrags();
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
	    drawRockets();
	    drawParticles();

	    if(base->getHp() <= 0) {
	    	std::cout << "Base2 hp: " << base2->getHp() << std::endl;
	    	std::cout << "Team 2 WON!" << std::endl;
	    	break;
	    }
	    else if(base2->getHp() <= 0) {
	    	std::cout << "Team 1 WON!. THEIR ROBOTS DID A GREAT JOB!!!" << std::endl;
	    	break;
	    }


		window->display();

	}

}


