/*
 * Engine.h
 *
 *  Created on: Mar 24, 2020
 *      Author: Eren
 */

#ifndef ENGINE_H_
#define ENGINE_H_

#include <SFML/Graphics.hpp>
#include <Box2D/Box2D.h>
#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include "ObjectFactory.h"
#include "Robot.h"
#include "Rocket.h"
#include "Base.h"
#include "Particle.h"

class Engine {
private:
	sf::RenderWindow *window;
	sf::Color windowColor;
	sf::Vector2f oldPos;
	float scale;
	float wwidth;
	float wheight;
	float zoom = 1;
	float gunAng1;
	float gunAng2;
	float timeStep;
	bool moving;
	b2Vec2 gravity;
	b2World *world;
	int32 velocityIterations;
	int32 positionIterations;
	ObjectFactory *objectFactory;
	std::vector<Object*> objects;
	std::vector<Robot*> robots;
	std::vector<Base*> bases;
	std::vector<Rocket*> rockets;
	std::vector<Particle*> particles;

	void readMainSettings();

	void split(std::string const &str, const char delim,std::vector<std::string> &out);
	b2Vec2 convertWorldToScreen( const b2Vec2& v);
	b2Vec2 convertScreenToWorld( const b2Vec2& v);

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
	void addParticle(Particle *p);
	void drawParticles();
	void clearDeadBodies();
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
