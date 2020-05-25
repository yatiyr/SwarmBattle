/*
 * Engine.h
 *
 *  Created on: Mar 24, 2020
 *      Author: Eren
 */

#include <SFML/Graphics.hpp>
#include <box2d/box2d.h>
#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include "ObjectFactory.h"
#include "Robot.h"
#include "Rocket.h"
#include "Base.h"

#ifndef ENGINE_H_
#define ENGINE_H_

class Engine {
private:
	sf::RenderWindow *window;
	sf::Color windowColor;
	float scale;
	float wwidth;
	float wheight;
	b2Vec2 gravity;

	float timeStep;
	int32 velocityIterations;
	int32 positionIterations;

	sf::Vector2f oldPos;
	bool moving;
	float zoom = 1;

	float gunAng1;
	float gunAng2;

	b2World *world;

	ObjectFactory *objectFactory;

	std::vector<Object*> objects;
	std::vector<Robot*> robots;
	std::vector<Base*> bases;
	std::vector<Rocket*> rockets;

	void readMainSettings();

	void split(std::string const &str, const char delim,
			std::vector<std::string> &out) {
		size_t start;
		size_t end = 0;

		while((start = str.find_first_not_of(delim, end)) != std::string::npos) {
			end = str.find(delim, start);
			out.push_back(str.substr(start, end-start));
		}

	}

	b2Vec2 convertWorldToScreen( const b2Vec2& v) {
		return b2Vec2( (v.x * scale + wwidth/2) , -v.y * scale + wheight/4);
	}

	b2Vec2 convertScreenToWorld( const b2Vec2& v) {
		return b2Vec2( ((v.x - wwidth/2)/scale), -(v.y - wheight/4)/scale);
	}

	void handleBase(Base *b);
	void handleBases();
	void handleRocketDrags();

public:
	Engine();
	virtual ~Engine();

	void addRobot(Robot *r);
	void actRobots();
	void drawRobots();
	void addObject(Object *obj);
	void actObjects();
	void drawObjects();
	void addBase(Base *b);
	void drawBases();
	void addRocket(Rocket *r);
	void drawRockets();

	const float getScale() { return scale; }
	const float getWwidth() { return wwidth; }
	const float getWheight() { return wheight; }
	const float getTimeStep() { return timeStep; }
	const int32 getVelocityIterations() { return velocityIterations; }
	const int32 getPositionIterations() { return positionIterations; }
	sf::RenderWindow *getWindow() { return window; }
	b2World *getWorld() { return world;}

	void run();


};

#endif /* ENGINE_H_ */
