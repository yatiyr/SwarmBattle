/*
 * ObjectEnums.h
 *
 *  Created on: Apr 18, 2020
 *      Author: Eren
 */

#ifndef OBJECTENUMS_H_
#define OBJECTENUMS_H_

enum entityCategory {
	OTHER = 0x0001,
	ROBOT_T1 = 0x0002,
	ROBOT_T2 = 0x0004,
	ROBOT_SENSOR_T1 = 0x0008,
	ROBOT_SENSOR_T2 = 0x0010,
	ROCKET_T1 = 0x0020,
	ROCKET_T2 = 0x0040,
	BASE_T1 = 0x0080,
	BASE_T2 = 0x0100,
	PARTICLE = 0x0200,
	GUN = 0x0400
};



#endif /* OBJECTENUMS_H_ */
