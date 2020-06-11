/*
 * Object.cpp
 *
 *  Created on: Jun 11, 2020
 *      Author: Eren
 */

#include <Object.h>


b2Vec2 Object::convertWorldToScreen( const b2Vec2& v, float scale, int wwidth, int wheight) {
  return b2Vec2( (v.x * scale + wwidth/2) , -v.y * scale + wheight/4);
}

b2Vec2 Object::convertScreenToWorld( const b2Vec2& v, float scale, int wwidth, int wheight) {
	return b2Vec2( ((v.x - wwidth/2)/scale), -(v.y - wheight/4)/scale);
}

void Object::bindWorld(b2World *world) {
	worldPointer = world;
}

void Object::bindWindow(sf::RenderWindow *window) {
	windowPointer = window;
}

Object::Object(b2Vec2 pos, float scale, int wwidth, int wheight) {
	b2Vec2 screenPos = convertScreenToWorld(pos, scale, wwidth, wheight);
	bodyDef.position.Set(screenPos.x, screenPos.y);
}

Object::Object(sf::RenderWindow *window, b2World *world, b2Vec2 pos, float scale, int wwidth, int wheight) {
	bodyDef.position.Set(pos.x, pos.y);
	worldPointer = world;
	windowPointer = window;
	body = 0;
	id = 0;
	teamId = 0;
}


void Object::draw(float scale, int wwidth, int wheight) {
	b2Vec2 pos = body->GetPosition();
	float angle = body->GetAngle();
	pos = convertWorldToScreen(pos, scale, wwidth, wheight);
	drawShape.setPosition(pos.x,pos.y);
	drawShape.setRotation(-(180/b2_pi) * angle);
	windowPointer->draw(drawShape);
}


