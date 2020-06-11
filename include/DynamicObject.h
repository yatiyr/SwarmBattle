/*
 * DynamicObject.h
 *
 *  Created on: Mar 26, 2020
 *      Author: Eren
 */

#include <Object.h>

#ifndef DYNAMICOBJECT_H_
#define DYNAMICOBJECT_H_

class DynamicObject : public Object {
protected:
	float hp;
	float damage;
public:
	DynamicObject(b2Vec2 pos, float scale, int wwidth, int wheight);
	DynamicObject(sf::RenderWindow *window, b2World *world, b2Vec2 pos, float scale, int wwidth, int wheight, int teamId);
	virtual ~DynamicObject() {}
	void setHp(float h) {hp = h;}
	float getHp() {return hp;}
	float getDamage() {return damage;}
};

#endif /* DYNAMICOBJECT_H_ */
