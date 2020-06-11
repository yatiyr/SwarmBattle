/*
 * Base.h
 *
 *  Created on: May 23, 2020
 *      Author: Eren
 */

#ifndef SRC_INCLUDE_BASE_H_
#define SRC_INCLUDE_BASE_H_

#include <StaticObject.h>
#include <ObjectEnums.h>
#include <iostream>

#define EPSILON 0.0005
#define DEGTORAD b2_pi/180

enum BState {
	GunSteady,
	Reloading,
	Adjusting,
	GunFiring
};

class ObjectFactory;

class Base : public StaticObject {
private:
	float hp;
	float gunOrientation;
	sf::Color bColor;
	b2Vec2 enemyBasePos;
	sf::ConvexShape gunDrawShape;
	b2Body *gunBody;
	b2BodyDef gunBodyDef;
	b2PolygonShape gunShape;
	b2FixtureDef gunFixtureDef;
	b2RevoluteJoint *baseJoint;

	Rocket *reloadedRocket = NULL;

	BState state;

public:
	Base(sf::RenderWindow *window, b2World *world, b2Vec2 pos, float scale, int wwidth, int wheight, sf::Color color, int teamId) : StaticObject(window,world,pos,scale,wwidth,wheight,teamId) {
		hp = 5000;
		gunOrientation = 0;
		gunBodyDef.type = b2_dynamicBody;
		bColor = color;
		drawShape.setPointCount(5);
		drawShape.setPoint(0, sf::Vector2f(0.f * scale, 0.f * scale));
		drawShape.setPoint(1, sf::Vector2f(0.f * scale, 30.f * scale));
		drawShape.setPoint(2, sf::Vector2f(20.f * scale, 45.f * scale));
		drawShape.setPoint(3, sf::Vector2f(40.f * scale, 30.f * scale));
		drawShape.setPoint(4, sf::Vector2f(40.f * scale, 0.f * scale));
		drawShape.setOrigin(20.f * scale,15.f * scale);
		drawShape.setFillColor(color);

		gunDrawShape.setPointCount(4);
		gunDrawShape.setPoint(0, sf::Vector2f(0.f * scale, 0.f * scale));
		gunDrawShape.setPoint(1, sf::Vector2f(0.f * scale, 9.f * scale));
		gunDrawShape.setPoint(2, sf::Vector2f(3.f * scale, 9.f * scale));
		gunDrawShape.setPoint(3, sf::Vector2f(3.f * scale, 0.f * scale));
		gunDrawShape.setOrigin(1.5f * scale, 4.5f * scale);
		gunDrawShape.setFillColor(sf::Color::Transparent);
		gunDrawShape.setOutlineThickness(0.6f);
		gunDrawShape.setOutlineColor(sf::Color::Black);

		gunBodyDef.position.Set(pos.x,pos.y + 34.f);
		gunBody = world->CreateBody(&gunBodyDef);
		gunBody->SetUserData(this);
		b2Vec2 vecArrayGun[4];
		vecArrayGun[0].Set(-1.5f,4.5f);
		vecArrayGun[1].Set(-1.5f,-4.5f);
		vecArrayGun[2].Set(1.5f,-4.5f);
		vecArrayGun[3].Set(1.5f,4.5f);
		gunShape.Set(vecArrayGun,4);
		gunFixtureDef.shape = &gunShape;
		gunFixtureDef.density = 1.0f;
		gunFixtureDef.friction = 30.f;
		gunFixtureDef.isSensor = true;
		gunFixtureDef.filter.categoryBits = entityCategory::GUN;
		// collision filterlar gelecek buraya
		gunBody->CreateFixture(&gunFixtureDef);


		body = world->CreateBody(&bodyDef);
		body->SetUserData(this);
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

		if(teamId == 0) {
			fixtureDef.filter.categoryBits = entityCategory::BASE_T1;
		}
		else if(teamId == 1) {
			fixtureDef.filter.categoryBits = entityCategory::BASE_T2;
		}

		body->CreateFixture(&fixtureDef);

		// Joint
		b2RevoluteJointDef revoluteJointDef;
		revoluteJointDef.bodyA = body;
		revoluteJointDef.bodyB = gunBody;
		revoluteJointDef.collideConnected = false;
		revoluteJointDef.localAnchorA.Set(0.f,30.f);
		revoluteJointDef.localAnchorB.Set(0.f,-4.f);
		revoluteJointDef.enableLimit = true;
		revoluteJointDef.lowerAngle = -85 * DEGTORAD;
		revoluteJointDef.upperAngle =  85 * DEGTORAD;
		revoluteJointDef.enableMotor = true;
		revoluteJointDef.maxMotorTorque = 20;
		revoluteJointDef.motorSpeed = 360 * DEGTORAD;
		baseJoint = (b2RevoluteJoint*) world->CreateJoint(&revoluteJointDef);

		state = BState::Reloading;

	}

	void draw(float scale, int wwidth, int wheight) {

		b2Vec2 pos = body->GetPosition();
		float angle = body->GetAngle();

		b2Vec2 gunPos = gunBody->GetPosition();
		float gunAngle = gunBody->GetAngle();

		pos = convertWorldToScreen(pos, scale, wwidth, wheight);
		gunPos = convertWorldToScreen(gunPos, scale, wwidth, wheight);

		// Bad solution by adding 180
		drawShape.setPosition(pos.x, pos.y);
		drawShape.setRotation((180/b2_pi) * angle + 180);

		gunDrawShape.setPosition(gunPos.x,gunPos.y);
		gunDrawShape.setRotation(-(180/b2_pi) * gunAngle);
		windowPointer->draw(drawShape);
		windowPointer->draw(gunDrawShape);

	}

	void act() {

	}

	sf::Color getColor() {
		return bColor;
	}


	b2Body *getGunBody() {
			return gunBody;
	}

	BState getState() {
		return state;
	}

	void setState(BState s) {
		state = s;
	}

	void setReloadedRocket(Rocket *r) {
		reloadedRocket = r;
	}

	Rocket *getReloadedRocket() {
		return reloadedRocket;
	}

	int getTeamId() {
		return teamId;
	}

	int getHp() {
		return hp;
	}

	void setHp(float h) {
		hp = h;
	}

	virtual ~Base() {
		body->GetWorld()->DestroyBody(body);
	}
};

#endif /* SRC_INCLUDE_BASE_H_ */
