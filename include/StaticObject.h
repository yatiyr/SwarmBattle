/*
 * StaticObject.h
 *
 *  Created on: Mar 26, 2020
 *      Author: Eren
 */

#include "Object.h"

#ifndef STATICOBJECT_H_
#define STATICOBJECT_H_

class StaticObject : public Object {
public:
	StaticObject(b2Vec2 pos, float scale, int wwidth, int wheight);
	StaticObject(sf::RenderWindow *window, b2World *world, b2Vec2 pos, float scale, int wwidth, int wheight, int teamId);
	virtual ~StaticObject() {}
};

#endif /* STATICOBJECT_H_ */