/*
 * DynamicObject.cpp
 *
 *  Created on: Jun 11, 2020
 *      Author: Eren
 */

#include <DynamicObject.h>

DynamicObject::DynamicObject(b2Vec2 pos, float scale, int wwidth, int wheight) : Object(pos,scale,wwidth,wheight) {
	bodyDef.type = b2_dynamicBody;
	hp = 0;
	damage = 0;
}

DynamicObject::DynamicObject(sf::RenderWindow *window, b2World *world, b2Vec2 pos, float scale, int wwidth, int wheight, int teamId) : Object(window,world, pos,scale,wwidth,wheight) {
	bodyDef.type = b2_dynamicBody;
	this->teamId = teamId;
	hp = 0;
	damage = 0;
}


