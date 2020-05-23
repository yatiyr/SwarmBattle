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

Rocket* ObjectFactory::createRocket(b2Vec2 pos, sf::Color color) {
	Rocket *result = new Rocket(window, world, pos, scale, wwidth, wheight, color);
	return result;
}

Robot* ObjectFactory::createRobot(b2Vec2 pos, sf::Color color) {
	Robot *result = new Robot(window, world, pos, scale, wwidth, wheight, color);
	return result;
}

