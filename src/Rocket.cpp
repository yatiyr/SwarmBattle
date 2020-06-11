/*
 * Rocket.cpp
 *
 *  Created on: Jun 11, 2020
 *      Author: Eren
 */

#include <Rocket.h>
#include <ObjectEnums.h>

Rocket::Rocket(sf::RenderWindow *window, b2World *world, b2Vec2 pos, float scale, int wwidth, int wheight, sf::Color color, int teamId) : Projectile(window,world,pos,scale,wwidth, wheight,teamId) {
	hp = 100;
	damage = 100;
	bColor = color;

	drawShape.setPointCount(5);
	drawShape.setPoint(0, sf::Vector2f(0.f * scale, 0.f * scale));
	drawShape.setPoint(1, sf::Vector2f(0.f * scale, 8.f * scale));
	drawShape.setPoint(2, sf::Vector2f(1.f * scale, 10.f * scale));
	drawShape.setPoint(3, sf::Vector2f(2.f * scale, 8.f * scale));
	drawShape.setPoint(4, sf::Vector2f(2.f * scale, 0.f * scale));
	drawShape.setOrigin(1.f * scale,8 * scale);
	drawShape.setFillColor(color);

//		body->SetEnabled(false);
	body = worldPointer->CreateBody(&bodyDef);
	body->SetUserData(this);
	body->SetEnabled(false);

	b2Vec2 verArray[5];
	verArray[0].Set(-1.f,-8.f);
	verArray[1].Set(1.f,-8.f);
	verArray[2].Set(1.f, 0.f);
	verArray[3].Set(0.f,2.f);
	verArray[4].Set(-1.f,0.f);
	shape.Set(verArray, 5);
	fixtureDef.shape = &shape;
	fixtureDef.density = 10.0f;
	fixtureDef.friction = 30.f;
	if(teamId == 0) {
		fixtureDef.filter.categoryBits = entityCategory::ROCKET_T1;
		fixtureDef.filter.maskBits = entityCategory::BASE_T2 |
									 entityCategory::ROCKET_T2 |
									 entityCategory::ROBOT_T2 |
									 entityCategory::PARTICLE |
									 entityCategory::OTHER;
	}
	else if(teamId == 1) {
		fixtureDef.filter.categoryBits = entityCategory::ROCKET_T2;
		fixtureDef.filter.maskBits = entityCategory::BASE_T1 |
									 entityCategory::ROCKET_T1 |
									 entityCategory::ROBOT_T1 |
									 entityCategory::PARTICLE |
									 entityCategory::OTHER;
	}
	body->CreateFixture(&fixtureDef);

	b2CircleShape rocketTip;
	b2FixtureDef rocketTipFixtureDef;

	rocketTip.m_radius = 1.0f;
	rocketTipFixtureDef.density = 1.f;
	rocketTipFixtureDef.friction = 30.f;
	rocketTipFixtureDef.shape = &rocketTip;
	rocketTipFixtureDef.isSensor = false;
	rocketTipFixtureDef.filter.maskBits = 0;
	body->CreateFixture(&rocketTipFixtureDef);
}

void Rocket::handleDrag() {
	b2Vec2 pointingDirection = body->GetWorldVector(b2Vec2(0.f,0.f));
	b2Vec2 flightDirection = body->GetLinearVelocity();
	float flightSpeed = flightDirection.Normalize();

	float dot = b2Dot(flightDirection, pointingDirection);
	float dragForceMagnitude = (1 - fabs(dot)) * flightSpeed * flightSpeed * 0.00001 * body->GetMass();

	b2Vec2 rocketTailPosition = body->GetWorldPoint(b2Vec2(0.f, -7.f));
	body->ApplyForce( dragForceMagnitude * -flightDirection, rocketTailPosition, true);
}

void Rocket::draw(float scale, int wwidth, int wheight) {
	b2Vec2 pos = body->GetPosition();
	float angle = body->GetAngle();
	pos = convertWorldToScreen(pos, scale, wwidth, wheight);
	drawShape.setPosition(pos.x,pos.y);
	drawShape.setRotation(-(180/b2_pi) * angle + 180);
	windowPointer->draw(drawShape);
}



