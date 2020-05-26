/*
 * Particle.h
 *
 *  Created on: May 26, 2020
 *      Author: Eren
 */

#ifndef SRC_INCLUDE_PARTICLE_H_
#define SRC_INCLUDE_PARTICLE_H_

#include <Projectile.h>

class Particle : public Projectile {
private:
	float blastImpactDamage;
public:
	Particle(sf::RenderWindow *window, b2World *world, b2Vec2 pos, float scale, int wwidth, int wheight, sf::Color color, int teamId) : Projectile(window,world,pos,scale,wwidth, wheight,teamId) {

		blastImpactDamage = 5;

		drawShape.setPointCount(4);
		drawShape.setPoint(0, sf::Vector2f(0.f * scale, 0.f * scale));
		drawShape.setPoint(1, sf::Vector2f(0.05f * scale, 0.07f * scale));
		drawShape.setPoint(2, sf::Vector2f(0.1f * scale, 0.f * scale));
		drawShape.setPoint(3, sf::Vector2f(0.05f * scale, -0.07f * scale));
		drawShape.setOrigin(0.05f * scale, 0.f * scale);
		drawShape.setFillColor(color);

		body = worldPointer->CreateBody(&bodyDef);
		body->SetUserData(this);

		b2Vec2 verArray[4];
		verArray[0].Set(-0.05f, 0.f);
		verArray[1].Set(0.f, -0.07f);
		verArray[2].Set(0.05f, 0.f);
		verArray[3].Set(0.f, 0.07f);
		shape.Set(verArray, 5);
		fixtureDef.shape = &shape;
		fixtureDef.density = 1.f;
		fixtureDef.friction = 30.f;

		fixtureDef.filter.categoryBits = entityCategory::PARTICLE;

	}

	void act() {

	}

	virtual ~Particle() {
		body->GetWorld()->DestroyBody(body);
	}
};

#endif /* SRC_INCLUDE_PARTICLE_H_ */
