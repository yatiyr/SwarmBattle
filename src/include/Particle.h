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
	sf::Color bColor;
public:
	Particle(sf::RenderWindow *window, b2World *world, b2Vec2 pos, float scale, int wwidth, int wheight, sf::Color color, int teamId) : Projectile(window,world,pos,scale,wwidth, wheight,teamId) {

		hp = 0.35f;
		damage = 0.1;
		bColor = color;

		bodyDef.bullet = true;
		bodyDef.linearDamping = 0;
		bodyDef.gravityScale = 2.5f;


		drawShape.setPointCount(4);
		drawShape.setPoint(0, sf::Vector2f(0.f * scale, 0.f * scale));
		drawShape.setPoint(1, sf::Vector2f(0.4f * scale, 0.56f * scale));
		drawShape.setPoint(2, sf::Vector2f(0.8f * scale, 0.f * scale));
		drawShape.setPoint(3, sf::Vector2f(0.3f * scale, -0.56f * scale));
		drawShape.setOrigin(0.2f * scale, 0.f * scale);
		drawShape.setFillColor(color);

		body = worldPointer->CreateBody(&bodyDef);
		body->SetUserData(this);

		b2Vec2 verArray[4];
		verArray[0].Set(-0.4f, 0.f);
		verArray[1].Set(0.f, -0.56f);
		verArray[2].Set(0.4f, 0.f);
		verArray[3].Set(0.f, 0.56f);
		shape.Set(verArray, 5);
		fixtureDef.shape = &shape;
		fixtureDef.density = 1.f;
		fixtureDef.friction = 1.f;
		fixtureDef.restitution = 0.1f;

		fixtureDef.filter.categoryBits = entityCategory::PARTICLE;
		fixtureDef.filter.maskBits = entityCategory::BASE_T1 | entityCategory::BASE_T2 |
									 entityCategory::ROCKET_T1 | entityCategory::ROCKET_T2 |
									 entityCategory::ROBOT_T1 | entityCategory::ROBOT_T2 |
									 entityCategory::OTHER;
		body->CreateFixture(&fixtureDef);
	}

	sf::Color getColor() {
		return bColor;
	}

	int getTeamId() {
		return teamId;
	}

	void act() {

	}

	virtual ~Particle() {
		body->GetWorld()->DestroyBody(body);
	}
};

#endif /* SRC_INCLUDE_PARTICLE_H_ */
