/*
 * Base.h
 *
 *  Created on: May 23, 2020
 *      Author: Eren
 */

#ifndef SRC_INCLUDE_BASE_H_
#define SRC_INCLUDE_BASE_H_

#include <StaticObject.h>

class Base : public StaticObject {
private:
	float hp;
	float gunOrientation;

	b2Vec2 enemyBasePos;

public:
	Base(sf::RenderWindow *window, b2World *world, b2Vec2 pos, float scale, int wwidth, int wheight, sf::Color color, int teamId) : StaticObject(window,world,pos,scale,wwidth,wheight,teamId) {
		hp = 5000;
		gunOrientation = 0;

		drawShape.setPointCount(5);
		drawShape.setPoint(0, sf::Vector2f(0.f * scale, 0.f * scale));
		drawShape.setPoint(1, sf::Vector2f(0.f * scale, 30.f * scale));
		drawShape.setPoint(2, sf::Vector2f(20.f * scale, 45.f * scale));
		drawShape.setPoint(3, sf::Vector2f(40.f * scale, 30.f * scale));
		drawShape.setPoint(4, sf::Vector2f(40.f * scale, 0.f * scale));
		drawShape.setOrigin(20.f * scale,15 * scale);
		drawShape.setFillColor(color);

		body = worldPointer->CreateBody(&bodyDef);

		b2Vec2 verArray[5];
		verArray[0].Set(-20.f,15.f);
		verArray[1].Set(-20.f,-15.f);
		verArray[2].Set(20.f, -15.f);
		verArray[3].Set(20.f,15.f);
		verArray[4].Set(0.f,30.f);
		shape.Set(verArray, 5);
		fixtureDef.shape = &shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 30.f;
		body->CreateFixture(&fixtureDef);

	}

	void act() {

	}
	virtual ~Base();
};

#endif /* SRC_INCLUDE_BASE_H_ */
