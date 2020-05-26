/*
 * ObjectFactory.cpp
 *
 *  Created on: Mar 26, 2020
 *      Author: Eren
 */

#include "ObjectFactory.h"

ObjectFactory::ObjectFactory(b2World *world, sf::RenderWindow *window, float scale, int wwidth, int wheight) {

	this->world = world;
	this->window = window;
	this->scale = scale;
	this->wwidth = wwidth;
	this->wheight = wheight;

}

ObjectFactory::~ObjectFactory() {}

Rocket* ObjectFactory::createRocket(b2Vec2 pos, sf::Color color, int teamId) {
	Rocket *result = new Rocket(window, world, pos, scale, wwidth, wheight, color, teamId);
	return result;
}

Robot* ObjectFactory::createRobot(b2Vec2 pos, sf::Color color, int teamId) {
	Robot *result = new Robot(window, world, pos, scale, wwidth, wheight, color, teamId);
	return result;
}

Base* ObjectFactory::createBase(b2Vec2 pos, sf::Color color, int teamId) {
	Base *result = new Base(window, world, pos, scale, wwidth, wheight, color, teamId);
	return result;
}

Particle* ObjectFactory::createParticle(b2Vec2 pos, sf::Color color, int teamId) {
	Particle *result = new Particle(window, world, pos, scale, wwidth, wheight, color ,teamId);
	return result;
}



