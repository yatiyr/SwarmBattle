/*
 * SensorContactListener.h
 *
 *  Created on: Apr 19, 2020
 *      Author: Eren
 */

#include <box2d/box2d.h>
#include "Robot.h"

#ifndef SENSORCONTACTLISTENER_H_
#define SENSORCONTACTLISTENER_H_

bool getSensedRobot(b2Contact* contact, Robot*& sensor1, Robot*& sensor2) {

	b2Fixture* fixtureA = contact->GetFixtureA();
	b2Fixture* fixtureB = contact->GetFixtureB();

	bool sensorA = fixtureA->IsSensor();
	bool sensorB = fixtureB->IsSensor();

	if(!(sensorA && sensorB))
		return false;

	Robot* entityA = static_cast<Robot*>(fixtureA->GetBody()->GetUserData());
	Robot* entityB = static_cast<Robot*>(fixtureB->GetBody()->GetUserData());

	sensor1 = entityA;
	sensor2 = entityB;

	return true;

}

bool getDynamicObjectCollision(b2Contact* contact, DynamicObject*& obj1, DynamicObject*& obj2) {
	b2Fixture* fixtureA = contact->GetFixtureA();
	b2Fixture* fixtureB = contact->GetFixtureB();

	int cbA = fixtureA->GetFilterData().categoryBits;
	int cbB = fixtureB->GetFilterData().categoryBits;
	if(!((cbA == entityCategory::ROBOT_T1 ||
		cbA == entityCategory::ROBOT_T2 ||
		cbA == entityCategory::ROCKET_T1 ||
		cbA == entityCategory::ROCKET_T2 ||
		cbA == entityCategory::PARTICLE) &&
	   (cbB == entityCategory::ROBOT_T1 ||
		cbB == entityCategory::ROBOT_T2 ||
		cbB == entityCategory::ROCKET_T1 ||
		cbB == entityCategory::ROCKET_T2 ||
		cbB == entityCategory::PARTICLE))) {
		return false;
	}

	DynamicObject* entityA = static_cast<DynamicObject*>(fixtureA->GetBody()->GetUserData());
	DynamicObject* entityB = static_cast<DynamicObject*>(fixtureB->GetBody()->GetUserData());

	obj1 = entityA;
	obj2 = entityB;

	return true;
}

bool getDynamicObjectOtherCollision(b2Contact* contact, DynamicObject*& obj) {

	b2Fixture* fixtureA = contact->GetFixtureA();
	b2Fixture* fixtureB = contact->GetFixtureB();

	int cbA = fixtureA->GetFilterData().categoryBits;
	int cbB = fixtureB->GetFilterData().categoryBits;

	if(!(((cbA == entityCategory::ROBOT_T1 ||
		cbA == entityCategory::ROBOT_T2 ||
		cbA == entityCategory::ROCKET_T1 ||
		cbA == entityCategory::ROCKET_T2 ||
		cbA == entityCategory::PARTICLE) && cbB == entityCategory::OTHER) ||
	  ((cbB == entityCategory::ROBOT_T1 ||
		cbB == entityCategory::ROBOT_T2 ||
		cbB == entityCategory::ROCKET_T1 ||
		cbB == entityCategory::ROCKET_T2 ||
		cbB == entityCategory::PARTICLE) && cbA == entityCategory::OTHER))) {

		return false;
	}

	if((cbA == entityCategory::ROBOT_T1 ||
		cbA == entityCategory::ROBOT_T2 ||
		cbA == entityCategory::ROCKET_T1 ||
		cbA == entityCategory::ROCKET_T2 ||
		cbA == entityCategory::PARTICLE) && cbB == entityCategory::OTHER) {

		DynamicObject* entity = static_cast<DynamicObject*>(fixtureA->GetBody()->GetUserData());
		obj = entity;
	}
	else if((cbB == entityCategory::ROBOT_T1 ||
		cbB == entityCategory::ROBOT_T2 ||
		cbB == entityCategory::ROCKET_T1 ||
		cbB == entityCategory::ROCKET_T2 ||
		cbB == entityCategory::PARTICLE) && cbA == entityCategory::OTHER) {

		DynamicObject* entity = static_cast<DynamicObject*>(fixtureB->GetBody()->GetUserData());
		obj = entity;
	}

	return true;

}

bool getDynamicObjectBaseCollision(b2Contact* contact, DynamicObject*& obj, Base*& base) {

	b2Fixture* fixtureA = contact->GetFixtureA();
	b2Fixture* fixtureB = contact->GetFixtureB();

	int cbA = fixtureA->GetFilterData().categoryBits;
	int cbB = fixtureB->GetFilterData().categoryBits;

	if(!(((cbA == entityCategory::ROBOT_T1 ||
		cbA == entityCategory::ROBOT_T2 ||
		cbA == entityCategory::ROCKET_T1 ||
		cbA == entityCategory::ROCKET_T2 ||
		cbA == entityCategory::PARTICLE) && (cbB == entityCategory::BASE_T1 || cbB == entityCategory::BASE_T2)) ||
	  ((cbB == entityCategory::ROBOT_T1 ||
		cbB == entityCategory::ROBOT_T2 ||
		cbB == entityCategory::ROCKET_T1 ||
		cbB == entityCategory::ROCKET_T2 ||
		cbB == entityCategory::PARTICLE) && (cbA == entityCategory::BASE_T1 || cbA == entityCategory::BASE_T2)))) {

		return false;
	}

	if((cbA == entityCategory::ROBOT_T1 ||
		cbA == entityCategory::ROBOT_T2 ||
		cbA == entityCategory::ROCKET_T1 ||
		cbA == entityCategory::ROCKET_T2 ||
		cbA == entityCategory::PARTICLE) && (cbB == entityCategory::BASE_T1 || cbB == entityCategory::BASE_T2)) {

		DynamicObject* entityA = static_cast<DynamicObject*>(fixtureA->GetBody()->GetUserData());
		Base* 		   entityB = static_cast<Base*>(fixtureA->GetBody()->GetUserData());
		obj = entityA;
		base = entityB;
	}
	else if((cbB == entityCategory::ROBOT_T1 ||
		cbB == entityCategory::ROBOT_T2 ||
		cbB == entityCategory::ROCKET_T1 ||
		cbB == entityCategory::ROCKET_T2 ||
		cbB == entityCategory::PARTICLE) && (cbA == entityCategory::BASE_T1 || cbA == entityCategory::BASE_T2)) {

		DynamicObject* entityB = static_cast<DynamicObject*>(fixtureB->GetBody()->GetUserData());
		Base* 		   entityA = static_cast<Base*>(fixtureA->GetBody()->GetUserData());
		obj = entityB;
		base = entityA;
	}

	return true;
}

class SensorContactListener : public b2ContactListener {


	void BeginContact(b2Contact* contact) {
		Robot* sensor1;
		Robot* sensor2;
		DynamicObject* obj1;
		DynamicObject* obj2;
		Base* base;
		if(getSensedRobot(contact, sensor1, sensor2)) {
			sensor1->sensorAcquiredRobot(sensor2);
			sensor2->sensorAcquiredRobot(sensor1);
		}
		else if(getDynamicObjectCollision(contact, obj1, obj2)) {
			int cb1 = contact->GetFixtureA()->GetFilterData().categoryBits;
			int cb2 = contact->GetFixtureB()->GetFilterData().categoryBits;

			obj1->setHp(obj1->getHp() - obj2->getDamage());
			obj2->setHp(obj2->getHp() - obj1->getDamage());

		}
		else if(getDynamicObjectOtherCollision(contact, obj1)) {

			obj1->setHp(obj1->getHp() - obj1->getDamage());
		}
		else if(getDynamicObjectBaseCollision(contact, obj1, base)) {

			obj1->setHp(obj1->getHp() - 100);
			base->setHp(base->getHp() - obj1->getDamage());
		}

	}

	void EndContact(b2Contact* contact) {
		Robot* sensor1;
		Robot* sensor2;
		if(getSensedRobot(contact, sensor1, sensor2)) {
			sensor1->sensorLostRobot(sensor2);
			sensor2->sensorLostRobot(sensor1);

		}
	}


};



#endif /* SENSORCONTACTLISTENER_H_ */
