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

// Object is the base class for all entities in the game
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

	b2Vec2 convertWorldToScreen( const b2Vec2& v, float scale, int wwidth, int wheight);
	b2Vec2 convertScreenToWorld( const b2Vec2& v, float scale, int wwidth, int wheight);
	void bindWorld(b2World *world);
	void bindWindow(sf::RenderWindow *window);
public:
	Object(b2Vec2 pos, float scale, int wwidth, int wheight);
	Object(sf::RenderWindow *window, b2World *world, b2Vec2 pos, float scale, int wwidth, int wheight);
	virtual ~Object() {}
	virtual void act() = 0;
	void draw(float scale, int wwidth, int wheight);
	int getId() {return id;}
	void setId(int val) {id = val;}
	sf::ConvexShape getDrawShape() {return drawShape;}
	sf::RenderWindow *getWindow() {return windowPointer;}
	b2BodyDef getBodyDef() {return bodyDef;}
	b2Body *getBody() {return body;}
	b2PolygonShape getShape() {return shape;}
	b2FixtureDef getFixtureDef() {return fixtureDef;}
	b2World *getWorld() {return worldPointer;}

};

#endif /* OBJECT_H_ */
