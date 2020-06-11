/* Box2D helloworld.cpp combined with SFML */
#include <iostream>
#include <SFML/Graphics.hpp>
#include <Box2D/Box2D.h>
#include "Engine.h"

static const float SCALE = 40.0; //10px = 1 meter
static const float WWIDTH= 1400.0; //screen width
static const float WHEIGHT= 1000.0; //screen width

b2Vec2 convertWorldToScreen( const b2Vec2& v) {
  return b2Vec2( (v.x * SCALE + WWIDTH/2) , -v.y * SCALE + WHEIGHT/4);
}

// This is a simple example of building and running a simulation
// using Box2D. Here we create a large ground box and a small dynamic
// box.
int main(int argc, char** argv)
{

	Engine engine;
	engine.run();

	return 0;
}
