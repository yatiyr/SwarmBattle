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
		verArray[0].Set(-1.f,-8.f);
		verArray[1].Set(1.f,-8.f);
		verArray[2].Set(1.f, 0.f);
		verArray[3].Set(0.f,2.f);
		verArray[4].Set(-1.f,0.f);
		shape.Set(verArray, 5);
		fixtureDef.shape = &shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 30.f;
		if(teamId == 0) {
			fixtureDef.filter.categoryBits = entityCategory::ROCKET_T1;
			fixtureDef.filter.maskBits = entityCategory::BASE_T2 |
										 entityCategory::ROCKET_T2 |
										 entityCategory::ROBOT_T2 |
										 entityCategory::OTHER;
		}
		else if(teamId == 1) {
			fixtureDef.filter.categoryBits = entityCategory::ROCKET_T2;
			fixtureDef.filter.maskBits = entityCategory::BASE_T1 |
										 entityCategory::ROCKET_T1 |
										 entityCategory::ROBOT_T1 |
										 entityCategory::OTHER;
		}
		body->CreateFixture(&fixtureDef);

		b2CircleShape rocketTip;
		b2FixtureDef rocketTipFixtureDef;

		rocketTip.m_radius = 1.0f;
		rocketTipFixtureDef.density = 0.1f;
		rocketTipFixtureDef.friction = 30.f;
		rocketTipFixtureDef.shape = &rocketTip;
		rocketTipFixtureDef.isSensor = false;
		rocketTipFixtureDef.filter.maskBits = 0;
		body->CreateFixture(&rocketTipFixtureDef);
	}

	void act() {

	}

	void handleDrag() {
		b2Vec2 pointingDirection = body->GetWorldVector(b2Vec2(0.f,0.f));
		b2Vec2 flightDirection = body->GetLinearVelocity();
		float flightSpeed = flightDirection.Normalize();

		float dot = b2Dot(flightDirection, pointingDirection);
		float dragForceMagnitude = (1 - fabs(dot)) * flightSpeed * flightSpeed * 0.00001 * body->GetMass();

		b2Vec2 rocketTailPosition = body->GetWorldPoint(b2Vec2(0.f, -7.f));
		body->ApplyForce( dragForceMagnitude * -flightDirection, rocketTailPosition, true);
	}

	void draw(float scale, int wwidth, int wheight) {
		b2Vec2 pos = body->GetPosition();
		float angle = body->GetAngle();
		pos = convertWorldToScreen(pos, scale, wwidth, wheight);
		drawShape.setPosition(pos.x,pos.y);
		drawShape.setRotation(-(180/b2_pi) * angle + 180);
		windowPointer->draw(drawShape);
	}
	virtual ~Rocket() {

	}
};

#endif /* ROCKET_H_ */
