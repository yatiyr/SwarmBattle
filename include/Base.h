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
#include <Rocket.h>

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
	Base(sf::RenderWindow *window, b2World *world, b2Vec2 pos, float scale, int wwidth, int wheight, sf::Color color, int teamId);

	void draw(float scale, int wwidth, int wheight);
	void act() {}
	sf::Color getColor() {return bColor;}
	b2Body *getGunBody() {return gunBody;}
	BState getState() {return state;}
	void setState(BState s) {state = s;}
	void setReloadedRocket(Rocket *r) {reloadedRocket = r;}
	Rocket *getReloadedRocket() {return reloadedRocket;}
	int getTeamId() {return teamId;}
	int getHp() {return hp;}
	void setHp(float h) {hp = h;}
	virtual ~Base() {body->GetWorld()->DestroyBody(body);}
};

#endif /* SRC_INCLUDE_BASE_H_ */
