/*
 * SensorContactListener.h
 *
 *  Created on: Apr 19, 2020
 *      Author: Eren
 */

#include <Box2D/Box2D.h>
#include "Robot.h"

#ifndef SENSORCONTACTLISTENER_H_
#define SENSORCONTACTLISTENER_H_

class SensorContactListener : public b2ContactListener {

	void BeginContact(b2Contact* contact);
	void EndContact(b2Contact* contact);


};



#endif /* SENSORCONTACTLISTENER_H_ */
