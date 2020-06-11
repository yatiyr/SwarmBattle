/*
 * StaticObject.cpp
 *
 *  Created on: Jun 11, 2020
 *      Author: Eren
 */

#include <StaticObject.h>

StaticObject::StaticObject(b2Vec2 pos, float scale, int wwidth, int wheight) : Object(pos,scale,wwidth,wheight) {
	bodyDef.type = b2_staticBody;
}

StaticObject::StaticObject(sf::RenderWindow *window, b2World *world, b2Vec2 pos, float scale, int wwidth, int wheight, int teamId) : Object(window,world, pos,scale,wwidth,wheight) {
	bodyDef.type = b2_staticBody;
	this->teamId = teamId;
}


