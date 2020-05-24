/*
 * DynamicObject.h
 *
 *  Created on: Mar 26, 2020
 *      Author: Eren
 */

#include "Object.h"

#ifndef DYNAMICOBJECT_H_
#define DYNAMICOBJECT_H_

class DynamicObject : public Object {
public:
	DynamicObject(b2Vec2 pos, float scale, int wwidth, int wheight) : Object(pos,scale,wwidth,wheight) {
		bodyDef.type = b2_dynamicBody;
	}
	DynamicObject(sf::RenderWindow *window, b2World *world, b2Vec2 pos, float scale, int wwidth, int wheight, int teamId) : Object(window,world, pos,scale,wwidth,wheight) {
		bodyDef.type = b2_dynamicBody;
		this->teamId = teamId;
	}
	virtual ~DynamicObject() {

	}

};

#endif /* DYNAMICOBJECT_H_ */
