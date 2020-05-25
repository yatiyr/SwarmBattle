/*
 * ObjectFactory.h
 *
 *  Created on: Mar 26, 2020
 *      Author: Eren
 */

#include <SFML/Graphics.hpp>
#include <box2d/box2d.h>
#include "Robot.h"
#include "Rocket.h"
#include "Base.h"

#ifndef OBJECTFACTORY_H_
#define OBJECTFACTORY_H_

class ObjectFactory {
private:
	b2World *world;
	sf::RenderWindow *window;
	float scale;
	float wwidth;
	float wheight;

public:
	ObjectFactory(b2World *world, sf::RenderWindow *window, float scale, int wwidth, int wheight);
	Rocket *createRocket(b2Vec2 pos, sf::Color color, int teamId);
	Robot *createRobot(b2Vec2 pos, sf::Color color, int teamId);
	Base *createBase(b2Vec2 pos, sf::Color color, int teamId);
	virtual ~ObjectFactory();
};

#endif /* OBJECTFACTORY_H_ */
