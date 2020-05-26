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
#include "ObjectEnums.h"
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
	int id;
	float hp;
	float damage;
	float fuel;
	State state;
	b2Vec2 moveTarget;
	b2Vec2 direction;
	b2Vec2 lastPos;
	int maxForce = 50.f;
	bool spinning;
	bool orienting;
	bool patrolling;
	float orientation;
	float viewAngle;

	float patrolRadius;
	b2Vec2 patrolCenter;

	b2CircleShape sensorShape;
	b2FixtureDef sensorFixtureDef;

	std::vector<Robot*> sensedRobots;

	float dotProduct(b2Vec2 vec1, b2Vec2 vec2) {
		return vec1.x*vec2.x + vec1.y*vec2.y;
	}

public:
	Robot(sf::RenderWindow *window, b2World *world, b2Vec2 pos, float scale, int wwidth, int wheight, sf::Color color, int teamId) : DynamicObject(window,world,pos,scale,wwidth,wheight,teamId) {
		id = 0;
		hp = 50;
		damage = 1;
		fuel = 100;

		drawShape.setFillColor(color);
		drawShape.setPointCount(6);

		drawShape.setPoint(0, sf::Vector2f(0.f * scale, 0.f * scale));
		drawShape.setPoint(1, sf::Vector2f(1.f * scale, 0.5f * scale));
		drawShape.setPoint(2, sf::Vector2f(2.f * scale, 0.5f * scale));
		drawShape.setPoint(3, sf::Vector2f(3.f * scale, 0.f * scale));
		drawShape.setPoint(4, sf::Vector2f(2.f * scale, -0.5f * scale));
		drawShape.setPoint(5, sf::Vector2f(1.f * scale, -0.5f * scale));
		drawShape.setOrigin(1.5f * scale, 0.f * scale);

		body = world->CreateBody(&bodyDef);
		body->SetUserData(this);
		b2Vec2 verArrayRobot[6];
		verArrayRobot[0].Set(-1.5f,0.f);
		verArrayRobot[1].Set(-0.5f,-0.5f);
		verArrayRobot[2].Set(0.5f,-0.5f);
		verArrayRobot[3].Set(1.5f,0.f);
		verArrayRobot[4].Set(0.5f,0.5f);
		verArrayRobot[5].Set(-0.5f, 0.5f);
		shape.Set(verArrayRobot,6);
		fixtureDef.shape = &shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 30.f;
//		fixtureDef.filter.maskBits = entityCategory::OTHER; //| entityCategory::ROBOT;

		if(teamId == 0) {
			fixtureDef.filter.categoryBits = entityCategory::ROBOT_T1;
			sensorFixtureDef.filter.categoryBits = entityCategory::ROBOT_SENSOR_T1;
			sensorFixtureDef.filter.maskBits = entityCategory::ROBOT_SENSOR_T1;
			fixtureDef.filter.maskBits = entityCategory::OTHER |
										 entityCategory::PARTICLE;
		}
		else if(teamId == 1) {
			fixtureDef.filter.categoryBits = entityCategory::ROBOT_T2;
			sensorFixtureDef.filter.categoryBits = entityCategory::ROBOT_SENSOR_T2;
			sensorFixtureDef.filter.maskBits = entityCategory::ROBOT_SENSOR_T2;
			fixtureDef.filter.maskBits = entityCategory::OTHER |
										 entityCategory::PARTICLE;
		}

		body->CreateFixture(&fixtureDef);

		sensorShape.m_radius = 32.0f;
		sensorFixtureDef.shape = &sensorShape;
		sensorFixtureDef.isSensor = true;
		body->CreateFixture(&sensorFixtureDef);




		state = State::Steady;
		spinning = false;
		orienting = false;
		orientation = 0;
		patrolling = true;
		direction.x = 0.0f;
		direction.y = 1.f;
		viewAngle = 120;
		patrolRadius = 0;

	}

	float getHp() { return hp; }
	float getDamage() { return damage; }
	float getFuel() { return damage; }

	void setHp(float val) { hp = val; }
	void setDamage(float val) { damage = val; }
	void setFuel(float val) { fuel = val; }

	void sensorAcquiredRobot(Robot *robot) {
		sensedRobots.push_back(robot);
	}

	void sensorLostRobot(Robot *robot) {
		sensedRobots.erase(std::find(sensedRobots.begin(), sensedRobots.end(), robot));
	}

	void orientationControl() {
		float nextAngle = body->GetAngle() + body->GetAngularVelocity()/60.0;
		float totalRotation = orientation - nextAngle;
		while(totalRotation < -180 * DEGTORAD)
			totalRotation += 360 * DEGTORAD;
		while(totalRotation > 180 * DEGTORAD)
			totalRotation -= 360 * DEGTORAD;
		float desiredAngularVelocity = totalRotation * 60.0f;
		int direction = 1;
		if(totalRotation < 0)
			direction = -1;
		else if(totalRotation == 0)
			direction = 0;

		float torque = body->GetInertia() * desiredAngularVelocity / (1/60.0f);
		body->ApplyTorque(torque, true);
	}

	void spinControl() {


		float angle = std::fmod(body->GetAngle()*(180/b2_pi),360);
		if(body->GetAngularVelocity() > 0) {
			body->ApplyTorque(-0.07f, true);
		}
		else {
			body->ApplyTorque(0.07f, true);
		}


	}

	void move(b2Vec2 targetPoint) {

		float targetSpeed = 1.f;

		b2Vec2 direction = targetPoint - body->GetPosition();
		float distanceToTravel = direction.Normalize();
		float speedToUse = targetSpeed;

		float distancePerTimeStep = speedToUse / 240.0f;
		if (distancePerTimeStep > distanceToTravel)
			speedToUse *= (distanceToTravel / distancePerTimeStep);

		b2Vec2 desiredVelocity = speedToUse * direction;
		b2Vec2 changeInVelocity = desiredVelocity - body->GetLinearVelocity();

		b2Vec2 force = body->GetMass() * 240.0f * changeInVelocity;
		b2Vec2 forceMax = maxForce*direction;

		float dist = (targetPoint - body->GetPosition()).Length();
		if(dist < EPSILON) {
			body->SetLinearVelocity(b2Vec2(0.f,0.f));
			state = State::Moving;
			return;
		}
		else {
			if(force.Length() > forceMax.Length() && dotProduct(direction, body->GetLinearVelocity()) < 0) {
				b2Vec2 velDir =  -body->GetLinearVelocity();
				float speed = velDir.Normalize();
				b2Vec2 stopForce = maxForce*velDir;
				body->ApplyForce(stopForce, body->GetWorldCenter(), true);
			}
			else if(force.Length() > forceMax.Length() && dotProduct(direction, body->GetLinearVelocity()) >= 0) {
				body->ApplyForce(forceMax, body->GetWorldCenter(), true);
			}
			else if(force.Length() <= forceMax.Length() && dotProduct(direction, body->GetLinearVelocity()) >= 0) {

				body->ApplyForce(force, body->GetWorldCenter(), true);
			}
		}
	}

	void moveToDirection(b2Vec2 direction) {

		// change orientation according to the direction
//		std::cout<< orientation << std::endl;
//		orientation = -atan(direction.x/direction.y);

		float targetSpeed = 2.f;
		b2Vec2 currentVelocity = body->GetLinearVelocity();
		float speed = currentVelocity.Length();
		b2Vec2 desiredVelocity = targetSpeed * direction;
		b2Vec2 velocityChange = desiredVelocity - currentVelocity;

		b2Vec2 force = body->GetMass() * 240.0f * velocityChange;
		velocityChange.Normalize();
		b2Vec2 forceMax = maxForce*velocityChange;

		if(force.Length() > forceMax.Length()) {
			body->ApplyForce(forceMax, body->GetWorldCenter(), true);
		}
		else if(force.Length() <= forceMax.Length()){
			body->ApplyForce(force, body->GetWorldCenter(), true);
		}


	}

	void changeVelocity(b2Vec2 vel) {

		b2Vec2 currentVelocity = body->GetLinearVelocity();
		b2Vec2 desiredVelocity = vel;
		b2Vec2 velocityChange = desiredVelocity - currentVelocity;

		b2Vec2 force = body->GetMass()*240.0f*velocityChange;
		velocityChange.Normalize();
		b2Vec2 forceMax = maxForce*velocityChange;

		if(force.Length() > forceMax.Length()) {
			body->ApplyForce(forceMax, body->GetWorldCenter(), true);
		}
		else if(force.Length() <= forceMax.Length()){
			body->ApplyForce(force, body->GetWorldCenter(), true);
		}

	}

	b2Vec2 turnDirection(float angle) {

		b2Vec2 result;
		float radAngle = angle*DEGTORAD;

		float cs = cos(radAngle);
		float sn = sin(radAngle);

		result.x = direction.x*cs - direction.y*sn;
		result.y = direction.x*sn + direction.y*cs;

		result.Normalize();

		return result;

	}

	float isInArea() {

		b2Vec2 pos = body->GetPosition();

		bool res = ((pos.x - patrolCenter.x)*(pos.x - patrolCenter.x) + (pos.y - patrolCenter.y)*(pos.y - patrolCenter.y)) <= patrolRadius*patrolRadius;

		if(res) {
			return ((pos.x - patrolCenter.x)*(pos.x - patrolCenter.x) + (pos.y - patrolCenter.y)*(pos.y - patrolCenter.y));
		}
		else
			return -1;

	}

	bool isRobotInViewField(Robot *robot) {

		b2Vec2 curVel = robot->body->GetLinearVelocity();
		curVel.Normalize();

		b2Vec2 point = robot->getBody()->GetPosition();
		b2Vec2 cPoint = body->GetPosition();

		b2Vec2 vec = point - cPoint;
		float distance = vec.Length();
		float angleBetween;
		if(curVel.Length() == 0)
			angleBetween = acos(dotProduct(vec,direction)/(direction.Length()*vec.Length()));
		else
			angleBetween = acos(dotProduct(vec,curVel)/(curVel.Length()*vec.Length()));

		float boundaryAngle = viewAngle;
		angleBetween = (180/b2_pi)*angleBetween;


		if(angleBetween > boundaryAngle || distance > sensorShape.m_radius)
			return false;
		else
			return true;

	}

	std::vector<Robot*> giveRobotsInArea() {

		std::vector<Robot*> result;

		for(int i=0;i<sensedRobots.size();i++) {
			if(isRobotInViewField(sensedRobots[i]))
				result.push_back(sensedRobots[i]);
		}

		return result;
	}


	// Boids algorithm rule methods

	// Rule1 Cohesion: Try to fly towards the center of mass
	// of neighboring robots
	b2Vec2 cohesion() {
		std::vector<Robot*> robotsInArea = giveRobotsInArea();
		b2Vec2 center(0.f,0.f);
		for(unsigned int i=0;i<robotsInArea.size();i++) {
			center += robotsInArea[i]->body->GetPosition();
		}
		center.x /= robotsInArea.size();
		center.y /= robotsInArea.size();

		// Move 1% of the way towards center
		b2Vec2 result;
		result = center - body->GetPosition();
		result.x /= 100;
		result.y /= 100;

		return result;
	}

	// Rule2 Separation: Try to keep small distance away from other
	// objects
	b2Vec2 separation() {

		b2Vec2 result(0.f,0.f);

		std::vector<Robot*> robotsInArea = giveRobotsInArea();
		for(unsigned int i=0;i<robotsInArea.size();i++) {
			b2Vec2 vec = robotsInArea[i]->body->GetPosition() - body->GetPosition();
			float distance = vec.Length();
			vec.x *= (1/distance*distance*distance)*10000;
			vec.y *= (1/distance*distance*distance)*10000;
			if(distance < 2.5) {
				result -= vec;
			}
		}

		return result;

	}

	b2Vec2 alignment() {

		b2Vec2 result(0.f,0.f);
		std::vector<Robot*> robotsInArea = giveRobotsInArea();
		for(unsigned int i=0;i<robotsInArea.size();i++) {
			result += robotsInArea[i]->body->GetLinearVelocity();
		}

		result.x /= robotsInArea.size();
		result.y /= robotsInArea.size();

		result -= body->GetLinearVelocity();
		result.x *= 16;
		result.y *= 16;

		return result;

	}

	b2Vec2 boidsAlgorithmVelocity() {

		b2Vec2 currentVelocity = body->GetLinearVelocity();

		// Computed values;
		b2Vec2 v1, v2, v3;


		// Take velocities from
		// boids algorithm main rules
		v1 = cohesion();
		v2 = separation();
		v3 = alignment();


		b2Vec2 desiredVelocity = currentVelocity + v1 + v2 + v3;

		return desiredVelocity;

	}

	void patrolArea() {

		srand(time(NULL));
		int dir = rand()%2;

		float targetSpeed = 50.f;
		b2Vec2 currentPosition = body->GetPosition();
		b2Vec2 currentVelocity = body->GetLinearVelocity();

		b2Vec2 steerVel = body->GetLinearVelocity();
		b2Vec2 steer = direction;
		std::vector<Robot*> robotsInArea = giveRobotsInArea();
//		b2Vec2 averagePoint;
//		b2Vec2 averageVel;
//		if(robotsInArea.size() > 0) {
////			steer = direction*steerVel.Length();
//			for(int i=0;i<robotsInArea.size();i++) {
//				Robot *r = robotsInArea[i];
//				b2Vec2 p = r->getBody()->GetPosition();
//				b2Vec2 v = r->getBody()->GetLinearVelocity();
//
//				averagePoint += p;
//				averageVel   += v;
//
//				b2Vec2 vec = p - currentPosition;
//				float dst = vec.Length();
//				vec.Normalize();
//				vec.x = (1/dst)*5*vec.x;
//				vec.y = (1/dst)*5*vec.y;
//				if(dst < 4.5f) {
//					moveToDirection(-vec);
//				}
//
//			}
////			averagePoint.x = averagePoint.x/robotsInArea.size();
////			averagePoint.y = averagePoint.y/robotsInArea.size();
////
////			b2Vec2 pVec = averagePoint - currentPosition;
////
////			averageVel  .x = averageVel  .x/robotsInArea.size();
////			averageVel  .y = averageVel  .y/robotsInArea.size();
////
////			averageVel = pVec+averageVel;
////			averageVel.Normalize();
////			direction = averageVel;
////			moveToDirection(direction);
//
////			return;
//
//
//		}

		if(isInArea() != -1) {
			int d = rand()%2;
			if(d == 0) {
				direction = turnDirection(0.5);
			}
			else if(d == 1) {
				direction = turnDirection(-0.5);
			}
		}
		else {
			b2Vec2 dist = patrolCenter - currentPosition;
			dist.Normalize();
			direction = dist;
		}

		b2Vec2 desiredVelocity;
//		std::vector<Robot*> robotsInArea = giveRobotsInArea();
		if(robotsInArea.size() > 0) {
			desiredVelocity = targetSpeed * direction;
			desiredVelocity += boidsAlgorithmVelocity();
			desiredVelocity.Normalize();
			desiredVelocity = targetSpeed*desiredVelocity;
		}
		else {
			desiredVelocity = targetSpeed * direction;
		}

		b2Vec2 velocityChange = desiredVelocity - currentVelocity;

		b2Vec2 force = body->GetMass() * 240.0f * velocityChange;
		velocityChange.Normalize();
		b2Vec2 forceMax = maxForce*velocityChange;

		if(force.Length() > forceMax.Length()) {
			body->ApplyForce(forceMax, body->GetWorldCenter(), true);
		}
		else if(force.Length() <= forceMax.Length()){
			body->ApplyForce(force, body->GetWorldCenter(), true);
		}


	}

	void draw(float scale, int wwidth, int wheight) {

		int drawFlag = 0;

		for(int i=0;i<sensedRobots.size();i++) {
			if(isRobotInViewField(sensedRobots[i]))
				drawFlag = 1;
		}

		if(drawFlag == 1) {
			drawShape.setFillColor(sf::Color::Red);
		}
		else {
			drawShape.setFillColor(sf::Color::Black);
		}

		b2Vec2 pos = body->GetPosition();
		float angle = body->GetAngle();
		pos = convertWorldToScreen(pos, scale, wwidth, wheight);
		drawShape.setPosition(pos.x,pos.y);
		drawShape.setRotation(-(180/b2_pi) * angle);
		windowPointer->draw(drawShape);
	}

	void act() {
		orientationControl();

		body->GetMass()*body->GetWorld()->GetGravity();
		body->ApplyForceToCenter(-body->GetMass()*body->GetWorld()->GetGravity(), true);
		if(state == State::Moving) {

			if(patrolling) {
				patrolCenter.x = 0.f;
				patrolCenter.y = 400.f;
				patrolRadius = 300.f;
				patrolArea();
//				changeVelocity(b2Vec2(1.f,1.f));
			}
			else {
			b2Vec2 targ(199.0f,0.f);
			b2Vec2 dir = targ - body->GetPosition();
			dir.Normalize();
		 	changeVelocity(dir);

			}
		}
		else if(state == State::Steady) {
			move(body->GetPosition());
		}


	}
	virtual ~Robot() {
		body->GetWorld()->DestroyBody(body);
	}
};

#endif /* ROBOT_H_ */
