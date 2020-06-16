/*
 * Robot.cpp
 *
 *  Created on: Jun 11, 2020
 *      Author: Eren
 */

#include <Robot.h>

/**
 * Robots are the agent which perform tasks in an air defence situation. Robots communicate with
 * each other and move as a swarm. Robots guard their bases and patrol in an area nearby the base.
 * They have two sensors, first sensor is a small one, which is for detecting their friends so that
 * they can arrange their velocities in the next time step to move as a swarm. Second sensor is for
 * detecting incoming rockets. When robots sense incoming rockets, they first compute estimate
 * their trajectories and if the threat level is high, they immediately lock on the rocket and try
 * to change their way. After their job is finished, they return back to the base. Their fuel is also
 * draining during the operations, so they also have to refuel themselves inside the base if they are
 * out of fuel.
 */
Robot::Robot(sf::RenderWindow *window, b2World *world, b2Vec2 pos,float timeStep, float scale, int wwidth, int wheight, sf::Color color, int teamId, b2Vec2 bl) : DynamicObject(window,world,pos,scale,wwidth,wheight,teamId) {
	id = 0;
	hp = 500;
	damage = 2;
	fuel = 100;
	bColor = color;
	baseLoc = bl;
	patrolDistanceMin = 220;
	patrolDistanceMax = 330;
	targetRocket = NULL;
	this->timeStep = timeStep;
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
	fixtureDef.density = 6.5f;
	fixtureDef.friction = 30.f;

	if(teamId == 0) {
		fixtureDef.filter.categoryBits = entityCategory::ROBOT_T1;
		sensorFixtureDef.filter.categoryBits = entityCategory::ROBOT_SENSOR_T1;
		sensorFixtureDef.filter.maskBits = entityCategory::ROBOT_SENSOR_T1;
		fixtureDef.filter.maskBits = entityCategory::OTHER |
									 entityCategory::PARTICLE |
									 entityCategory::ROCKET_T2;

		sensor2FixtureDef.filter.categoryBits = entityCategory::ROBOT_BSENSOR_T1;
		sensor2FixtureDef.filter.maskBits = entityCategory::ROCKET_T2;


	}
	else if(teamId == 1) {
		fixtureDef.filter.categoryBits = entityCategory::ROBOT_T2;
		sensorFixtureDef.filter.categoryBits = entityCategory::ROBOT_SENSOR_T2;
		sensorFixtureDef.filter.maskBits = entityCategory::ROBOT_SENSOR_T2;
		fixtureDef.filter.maskBits = entityCategory::OTHER |
									 entityCategory::PARTICLE |
									 entityCategory::ROCKET_T1;

		sensor2FixtureDef.filter.categoryBits = entityCategory::ROBOT_BSENSOR_T2;
		sensor2FixtureDef.filter.maskBits = entityCategory::ROCKET_T1;
	}

	body->CreateFixture(&fixtureDef);

	sensorShape.m_radius = 32.0f;
	sensorFixtureDef.shape = &sensorShape;
	sensorFixtureDef.isSensor = true;
	body->CreateFixture(&sensorFixtureDef);

	sensor2Shape.m_radius = 700.0f;
	sensor2FixtureDef.shape = &sensor2Shape;
	sensor2FixtureDef.isSensor = true;
	body->CreateFixture(&sensor2FixtureDef);




	state = State::Patrolling;
	spinning = false;
	orienting = false;
	returning = false;
	orientation = 0;
	patrolling = true;
	direction.x = 0.0f;
	direction.y = 1.f;
	viewAngle = 120;
	patrolRadius = 0;

}

float Robot::dotProduct(b2Vec2 vec1, b2Vec2 vec2) {
	return vec1.x*vec2.x + vec1.y*vec2.y;
}

/**
 * Since robots act in a fully pyhsical 2d environment, they
 * are under different external forces every time step which
 * causes unwanted torques so that they start spinning. At
 * the beginning of each time step, robots regain their balance
 * using this method.
 */
void Robot::orientationControl() {
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

void Robot::spinControl() {


	float angle = std::fmod(body->GetAngle()*(180/b2_pi),360);
	if(body->GetAngularVelocity() > 0) {
		body->ApplyTorque(-0.07f, true);
	}
	else {
		body->ApplyTorque(0.07f, true);
	}


}

/**
 * This method creates propulsion for robot to reach
 * a target point.
 */
void Robot::move(b2Vec2 targetPoint) {

	float targetSpeed = 30.f;

	if(state == State::Returning)
		targetSpeed = 90.f;

	b2Vec2 direction = targetPoint - body->GetPosition();
	float distanceToTravel = direction.Normalize();
	float speedToUse = targetSpeed;

	float distancePerTimeStep = speedToUse / 240.0f;
	if (distancePerTimeStep > distanceToTravel)
		speedToUse *= (distanceToTravel / distancePerTimeStep);

	b2Vec2 desiredVelocity = speedToUse * direction;
	b2Vec2 changeInVelocity = desiredVelocity - body->GetLinearVelocity();


	b2Vec2 force = body->GetMass() * 240.0f * changeInVelocity;
	changeInVelocity.Normalize();
	b2Vec2 forceMax = maxForce*changeInVelocity;

	b2Vec2 dir = targetPoint-body->GetPosition();
	float dd = dir.Length();
	if(dd < 1) {
		body->SetLinearVelocity(b2Vec2(0,0));
	}
	else {
		if(force.Length() > forceMax.Length()) {
			body->ApplyForce(forceMax, body->GetWorldCenter(), true);
		}
		else if(force.Length() <= forceMax.Length()){
			body->ApplyForce(force, body->GetWorldCenter(), true);
		}
	}



}

// If a robot has a target rocket,
// it chases it by creating propulsion
// towards it in each time step
void Robot::intercept(Rocket *r) {


	if(targetRocket != NULL) {
		b2Vec2 targetPoint = targetRocket->getBody()->GetPosition();

		float targetSpeed = 4000.f;
		body->SetLinearDamping(0);

		b2Vec2 direction = targetPoint - body->GetPosition();
		float distanceToTravel = direction.Normalize();
		float speedToUse = targetSpeed;

		float distancePerTimeStep = speedToUse / 240.0f;
		if (distancePerTimeStep > distanceToTravel)
			speedToUse *= (distanceToTravel / distancePerTimeStep);

		b2Vec2 desiredVelocity = speedToUse * direction;
		b2Vec2 changeInVelocity = desiredVelocity - body->GetLinearVelocity();

		b2Vec2 vel = body->GetLinearVelocity();

		float mF = 440;

		b2Vec2 force = body->GetMass() * 240.0f * changeInVelocity;
		changeInVelocity.Normalize();
		b2Vec2 forceMax = mF*changeInVelocity;

		b2Vec2 targetPos = targetRocket->getBody()->GetPosition();
		b2Vec2 distVec = body->GetPosition() - targetPos;
		float dist = distVec.Length();

		if(dist <= 5) {
			targetRocket->setRobotsIncoming(targetRocket->getRobotsIncoming() - 1);
			targetRocket = NULL;
			state = State::Returning;
			return;
		}

		b2Vec2 dir = targetPoint-body->GetPosition();
		if(force.Length() > forceMax.Length()) {
			body->ApplyLinearImpulse(forceMax, body->GetWorldCenter(), true);
		}
		else if(force.Length() <= forceMax.Length()){
			body->ApplyLinearImpulse(force, body->GetWorldCenter(), true);
		}

	}

	fuel -= 0.05;

}

/**
 * This method creates a propulsion along a direction.
 */
void Robot::moveToDirection(b2Vec2 direction) {

	// change orientation according to the direction
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

/**
 * If robot wants to change velocities, it uses this
 * method.
 */
void Robot::changeVelocity(b2Vec2 vel) {

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

/**
 * During patrolling, robots need to change their
 * directions randomly, this method changes its
 * turn direction.
 */
b2Vec2 Robot::turnDirection(float angle) {

	b2Vec2 result;
	float radAngle = angle*DEGTORAD;

	float cs = cos(radAngle);
	float sn = sin(radAngle);

	result.x = direction.x*cs - direction.y*sn;
	result.y = direction.x*sn + direction.y*cs;

	result.Normalize();

	return result;

}

/**
 * While patrolling, robots need to know whether they are
 * in the patrolling zone or not. This method answers their
 * question.
 */
float Robot::isInArea() {

	b2Vec2 pos = body->GetPosition();
	b2Vec2 v = baseLoc - pos;
	float distance = v.Length();

	if(teamId == 0) {
		if(distance < patrolDistanceMax && distance > patrolDistanceMin && pos.x >= baseLoc.x && pos.y > -180) {
			return 1;
		}
		else {
			return -1;
		}
	}
	else if(teamId == 1) {
		if(distance < patrolDistanceMax && distance > patrolDistanceMin && pos.x <= baseLoc.x && pos.y > -180) {
			return 1;
		}
		else {
			return -1;
		}
	}

	return -1;

//		bool res = ((pos.x - patrolCenter.x)*(pos.x - patrolCenter.x) + (pos.y - patrolCenter.y)*(pos.y - patrolCenter.y)) <= patrolRadius*patrolRadius;
//
//		if(res) {
//			return ((pos.x - patrolCenter.x)*(pos.x - patrolCenter.x) + (pos.y - patrolCenter.y)*(pos.y - patrolCenter.y));
//		}
//		else
//			return -1;

}

/**
 * For swarm behavior, robots sense other robots and change
 * their velocities according to them, this method helps them
 * sensing their robot friends.
 */
bool Robot::isRobotInViewField(Robot *robot) {

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

/**
 * Yields nearby friendly robots in an std::vector<Robot*>
 */
std::vector<Robot*> Robot::giveRobotsInArea() {

	std::vector<Robot*> result;

	for(int i=0;i<sensedRobots.size();i++) {
		if(isRobotInViewField(sensedRobots[i]))
			result.push_back(sensedRobots[i]);
	}

	return result;
}

/**
 * Targets a rocket to destroy
 */
void Robot::lockRocket(Rocket *r) {

	if(targetRocket == NULL) {
		if(r->getRobotsIncoming() < 4) {

			// If rocket trajectory is dangerous
			if(checkRocketTrajectory(r) == 1) {
				r->setRobotsIncoming(r->getRobotsIncoming() + 1);
				targetRocket = r;
			}
		}
	}
}


// Boids algorithm rule methods. Cohesion, Separation,
// Alignment and Boids algorithm velocity

// Rule1 Cohesion: Try to fly towards the center of mass
// of neighboring robots
b2Vec2 Robot::cohesion() {
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
b2Vec2 Robot::separation() {

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

// Rule3 Alignment: Robots align their velocities for a swarm
// behavior
b2Vec2 Robot::alignment() {

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

// This method computes a final velocity for robots by
// using results of Rule1, Rule2 and Rule3
b2Vec2 Robot::boidsAlgorithmVelocity() {

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

/**
 * Robots patrol an area if they are in a patrolling state.
 */
void Robot::patrolArea() {

	srand(time(NULL));
	int dir = rand()%2;

	float targetSpeed = 25.f;
	b2Vec2 currentPosition = body->GetPosition();
	b2Vec2 currentVelocity = body->GetLinearVelocity();

	b2Vec2 steerVel = body->GetLinearVelocity();
	b2Vec2 steer = direction;
	std::vector<Robot*> robotsInArea = giveRobotsInArea();

	if(isInArea() != -1) {
		int d = rand()%2;
		if(d == 0) {
			direction = turnDirection(0.6);
		}
		else if(d == 1) {
			direction = turnDirection(-0.6);
		}
	}
	else {
		b2Vec2 dist = baseLoc - currentPosition;
		float d = dist.Length();
		dist.Normalize();
		if(currentPosition.y < -40) {
			direction = b2Vec2(0.f,1.f);
		}
		else if(currentPosition.x < baseLoc.x) {
			direction = b2Vec2(1.f,0.f);
		}
		else if(d > patrolDistanceMax) {
			direction = dist;
		}
		else if(d < patrolDistanceMin) {
			direction = -dist;
		}
		else if(d > patrolDistanceMin && d < patrolDistanceMax) {
			direction = dist;
		}

	}

	b2Vec2 desiredVelocity;
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

	fuel -= 0.02;
}

/**
 * This is for drawing robot shape with sfml
 */
void Robot::draw(float scale, int wwidth, int wheight) {

	b2Vec2 pos = body->GetPosition();
	float angle = body->GetAngle();
	pos = convertWorldToScreen(pos, scale, wwidth, wheight);
	drawShape.setPosition(pos.x,pos.y);
	drawShape.setRotation(-(180/b2_pi) * angle);
	windowPointer->draw(drawShape);
}

/**
 * This method is called in each time step
 */
void Robot::act() {

	if(fuel < 10) {
		state = State::Refueling;
	}

	if(targetRocket != NULL) {
		state = State::Chasing;
	}

	if(state == State::Patrolling) {
		orientationControl();
		patrolArea();
		body->ApplyForceToCenter(-body->GetMass()*body->GetWorld()->GetGravity(), true);


	}
	else if(state == State::Chasing) {
		orientationControl();
		body->ApplyForceToCenter(-body->GetMass()*body->GetWorld()->GetGravity(), true);
		intercept(targetRocket);
	}
	else if(state == State::Refueling) {
		orientationControl();
		body->ApplyForceToCenter(-body->GetMass()*body->GetWorld()->GetGravity(), true);
		b2Vec2 dist = body->GetPosition() - baseLoc;
		float d = dist.Length();
		if(d > 50) {
			move(baseLoc);
		}
		else {
			move(baseLoc);
			refuel();
		}
	}
	else if(state == State::Steady) {

	}
	else if(state == State::Returning) {
		move(baseLoc);
		b2Vec2 distanceVec = baseLoc - body->GetPosition();
		float dis = distanceVec.Length();
		if(dis < 50) {
			state = State::Patrolling;
		}

	}


}

void Robot::refuel() {
	fuel += 0.3;
	if(fuel > 95) {
		state = State::Patrolling;
	}
}

Rocket *Robot::getTargetRocket() {
	return targetRocket;
}

/**
 * Robots estimate the trajectory of incoming robot
 * and decides whether it is going to explode near
 * the base. This is for determining for an interception
 * operation.
 *
 * This function returns 0 if trajectory of rocket does
 * not pose threat and 1 otherwise
 */
int Robot::checkRocketTrajectory(Rocket *r) {

	int step = 1;
	int result = 0;


	b2Vec2 rocketPosition = r->getBody()->GetPosition();
	b2Vec2 rocketVelocity = r->getBody()->GetLinearVelocity();
	b2Vec2 gravity = r->getBody()->GetWorld()->GetGravity();

	float dt = timeStep;
	b2Vec2 dv = dt * rocketVelocity;
	b2Vec2 da = dt * dt * gravity;

	// Terminal height for check Trajectory
	int threshold = -140;

	int m = 0;
	b2Vec2 pos(100000,100000);

	while(pos.y >= -180) {

		float s = step;
		pos = rocketPosition + s * dv + 0.5f * (s*s + s) * da;

		b2Vec2 distanceVec = baseLoc - pos;
		float dist = distanceVec.Length();

		if(dist <= 50) {
			std::cout <<pos.x<<" "<<pos.y<<" "<<dist <<std::endl;
			result = 1;
			break;
		}

		step++;
		m++;

	}

	return result;


}

