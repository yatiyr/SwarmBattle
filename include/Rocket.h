/*
 * Rocket.h
 *
 *  Created on: Mar 26, 2020
 *      Author: Eren
 */
#include "Projectile.h"

#ifndef ROCKET_H_
#define ROCKET_H_

class Rocket : public Projectile {
private:
	sf::Color bColor;

public:
	Rocket(sf::RenderWindow *window, b2World *world, b2Vec2 pos, float scale, int wwidth, int wheight, sf::Color color, int teamId);

	void act() {}
	void handleDrag();
	void draw(float scale, int wwidth, int wheight);

	sf::Color getColor() {return bColor;}
	int getTeamId() {return teamId;}
	virtual ~Rocket() {body->GetWorld()->DestroyBody(body);}

};

#endif /* ROCKET_H_ */
