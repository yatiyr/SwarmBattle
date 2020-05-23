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

class SensorContactListener : public b2ContactListener {


	void BeginContact(b2Contact* contact) {
		Robot* sensor1;
		Robot* sensor2;
		if(getSensedRobot(contact, sensor1, sensor2)) {
			sensor1->sensorAcquiredRobot(sensor2);
			sensor2->sensorAcquiredRobot(sensor1);
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
