/*
 * Object.h
 *
 *  Created on: Mar 25, 2020
 *      Author: Eren
 */

#include <SFML/Graphics.hpp>
#include <Box2D/Box2D.h>

#ifndef OBJECT_H_
#define OBJECT_H_

class Object {
private:

protected:
	int id;
	int teamId;
	sf::ConvexShape drawShape;
	sf::RenderWindow *windowPointer;

	b2BodyDef bodyDef;
	b2Body* body;
	b2PolygonShape shape;
	b2FixtureDef fixtureDef;

	b2World *worldPointer;

	b2Vec2 convertWorldToScreen( const b2Vec2& v, float scale, int wwidth, int wheight) {
	  return b2Vec2( (v.x * scale + wwidth/2) , -v.y * scale + wheight/4);
	}
	b2Vec2 convertScreenToWorld( const b2Vec2& v, float scale, int wwidth, int wheight) {
		return b2Vec2( ((v.x - wwidth/2)/scale), -(v.y - wheight/4)/scale);
	}
	void bindWorld(b2World *world) {
		worldPointer = world;
	}
	void bindWindow(sf::RenderWindow *window) {
		windowPointer = window;
	}

public:
	Object(b2Vec2 pos, float scale, int wwidth, int wheight) {
		b2Vec2 screenPos = convertScreenToWorld(pos, scale, wwidth, wheight);
		bodyDef.position.Set(screenPos.x, screenPos.y);
	}
	Object(sf::RenderWindow *window, b2World *world, b2Vec2 pos, float scale, int wwidth, int wheight) {
		bodyDef.position.Set(pos.x, pos.y);
		worldPointer = world;
		windowPointer = window;
		body = 0;
		id = 0;
		teamId = 0;
	}
	virtual ~Object() {

	}
	virtual void act() = 0;
	void draw(float scale, int wwidth, int wheight) {
		b2Vec2 pos = body->GetPosition();
		float angle = body->GetAngle();
		pos = convertWorldToScreen(pos, scale, wwidth, wheight);
		drawShape.setPosition(pos.x,pos.y);
		drawShape.setRotation(-(180/b2_pi) * angle);
		windowPointer->draw(drawShape);
	}

	int getId() {
		return id;
	}
	void setId(int val) {
		id = val;
	}

	sf::ConvexShape getDrawShape() {
		return drawShape;
	}

	sf::RenderWindow *getWindow() {
		return windowPointer;
	}

	b2BodyDef getBodyDef() {
		return bodyDef;
	}

	b2Body *getBody() {
		return body;
	}

	b2PolygonShape getShape() {
		return shape;
	}

	b2FixtureDef getFixtureDef() {
		return fixtureDef;
	}

	b2World *getWorld() {
		return worldPointer;
	}

};

#endif /* OBJECT_H_ */
