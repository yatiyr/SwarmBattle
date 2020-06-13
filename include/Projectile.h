/*
 * Projectile.h
 *
 *  Created on: Mar 26, 2020
 *      Author: Eren
 */

#ifndef PROJECTILE_H_
#define PROJECTILE_H_

#include "DynamicObject.h"

class Projectile : public DynamicObject {

public:
	Projectile(sf::RenderWindow *window, b2World *world, b2Vec2 pos, float scale, int wwidth, int wheight, int teamId) : DynamicObject(window,world,pos,scale,wwidth,wheight, teamId) {
		hp = 0;
		damage = 0;
	}

	virtual ~Projectile() {

	}
};

#endif /* PROJECTILE_H_ */
