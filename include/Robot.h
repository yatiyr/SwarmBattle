/*
 * Robot.h
 *
 *  Created on: Mar 26, 2020
 *      Author: Eren
 */
#include "DynamicObject.h"

#include <iostream>
#include <algorithm>
#include <math.h>
#include <time.h>
#include <ObjectEnums.h>
#include <vector>

#ifndef ROBOT_H_
#define ROBOT_H_

enum State {
	Moving,
	Steady,
	Patrolling
};

#define EPSILON 0.0005
#define DEGTORAD b2_pi/180

class Robot : public DynamicObject {
private:
	State state;
	int id;
	float maxForce = 120.f;
	float hp;
	float damage;
	float fuel;
	float orientation;
	float viewAngle;
	float patrolRadius;
	float patrolDistanceMin;
	float patrolDistanceMax;
	b2Vec2 moveTarget;
	b2Vec2 direction;
	b2Vec2 lastPos;
	b2Vec2 baseLoc;
	b2Vec2 patrolCenter;
	bool spinning;
	bool orienting;
	bool patrolling;

	b2CircleShape sensorShape;
	b2FixtureDef sensorFixtureDef;

	b2CircleShape sensor2Shape;
	b2FixtureDef sensor2FixtureDef;

	sf::Color bColor;


	std::vector<Robot*> sensedRobots;

	float dotProduct(b2Vec2 vec1, b2Vec2 vec2);
	float isInArea();
	void orientationControl();
	void spinControl();
	void move(b2Vec2 targetPoint);
	void moveToDirection(b2Vec2 direction);
	void changeVelocity(b2Vec2 vel);
	void patrolArea();
	bool isRobotInViewField(Robot *robot);
	std::vector<Robot*> giveRobotsInArea();

	b2Vec2 turnDirection(float angle);

	// Boids algorithm rule methods

	// Rule1 Cohesion: Try to fly towards the center of mass
	//				   of neighboring robots.
	b2Vec2 cohesion();

	// Rule2 Separation: Try to keep small distance away from other
	// 					 objects.
	b2Vec2 separation();

	// Rule3 Alignment: Robots align their velocities according to
	//					robots they see.
	b2Vec2 alignment();

	b2Vec2 boidsAlgorithmVelocity();

public:
	Robot(sf::RenderWindow *window, b2World *world, b2Vec2 pos, float scale, int wwidth, int wheight, sf::Color color, int teamId, b2Vec2 bl);

	sf::Color getColor() {return bColor;}
	virtual ~Robot() {body->GetWorld()->DestroyBody(body);}
	float getHp() { return hp; }
	float getDamage() { return damage; }
	float getFuel() { return damage; }
	void setHp(float val) { hp = val; }
	void setDamage(float val) { damage = val; }
	void setFuel(float val) { fuel = val; }
	int getTeamId() {return teamId;}
	void draw(float scale, int wwidth, int wheight);
	void act();
	void sensorAcquiredRobot(Robot *robot) {sensedRobots.push_back(robot);}
	void sensorLostRobot(Robot *robot) {sensedRobots.erase(std::find(sensedRobots.begin(), sensedRobots.end(), robot));}

};

#endif /* ROBOT_H_ */
