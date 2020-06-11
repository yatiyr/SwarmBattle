/*
 * Particle.h
 *
 *  Created on: May 26, 2020
 *      Author: Eren
 */

#ifndef PARTICLE_H_
#define PARTICLE_H_

#include <Projectile.h>

class Particle : public Projectile {
private:
	sf::Color bColor;
public:
	Particle(sf::RenderWindow *window, b2World *world, b2Vec2 pos, float scale, int wwidth, int wheight, sf::Color color, int teamId);

	sf::Color getColor() {return bColor;}
	int getTeamId() {return teamId;}
	void act() {}
	virtual ~Particle() {body->GetWorld()->DestroyBody(body);}
};

#endif /* SRC_INCLUDE_PARTICLE_H_ */
