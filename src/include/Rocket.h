/*
 * Rocket.h
 *
 *  Created on: Mar 26, 2020
 *      Author: Eren
 */
#include "Projectile.h"

#ifndef ROCKET_H_
#define ROCKET_H_

class Rocket : public Projectile {
public:
	Rocket(sf::RenderWindow *window, b2World *world, b2Vec2 pos, float scale, int wwidth, int wheight, sf::Color color, int teamId) : Projectile(window,world,pos,scale,wwidth, wheight,teamId) {
		hp = 100;
		damage = 10;

		drawShape.setPointCount(5);
		drawShape.setPoint(0, sf::Vector2f(0.f * scale, 0.f * scale));
		drawShape.setPoint(1, sf::Vector2f(0.f * scale, 8.f * scale));
		drawShape.setPoint(2, sf::Vector2f(1.f * scale, 10.f * scale));
		drawShape.setPoint(3, sf::Vector2f(2.f * scale, 8.f * scale));
		drawShape.setPoint(4, sf::Vector2f(2.f * scale, 0.f * scale));
		drawShape.setOrigin(1.f * scale,8 * scale);
		drawShape.setFillColor(color);

		body = worldPointer->CreateBody(&bodyDef);

		b2Vec2 verArray[5];
		verArray[0].Set(-1.f,8.f);
		verArray[1].Set(-1.f,0.f);
		verArray[2].Set(0.f, -2.f);
		verArray[3].Set(1.f,0.f);
		verArray[4].Set(1.f,8.f);
		shape.Set(verArray, 5);
		fixtureDef.shape = &shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 30.f;
		body->CreateFixture(&fixtureDef);

	}

	void act() {

	}
	virtual ~Rocket() {

	}
};

#endif /* ROCKET_H_ */
