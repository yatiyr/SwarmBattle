# SwarmBattle
Project for Ceng580

After I turned this project into a cmake project, I changed its file structure to make it cmake friendly. Include folder has header files and src has cpp files. In this simulation project we have classes;

* Object: All entities in the simulation except SensorContactListener and Engine derives from Object class. Object class implements simple box2d and sfml implementations. And with inheritence other classes use these constructors and easily wrap sfml and box2d.

* DynamicObject: They derive from Object class and their bodies are implemented as dynamic objects in box2d.

* StaticObject: They derive from Object class and their bodies are implemented as static objects in box2d.

* Base: Base derives from StaticObject class fires Rockets and has a certain hp(default 1000). It's box2d and sfml shapes and rocket firing functions are implemented. 

* Particle: Particle class derives from Dynamic Object and they are produced when a certain entity dies. It creates an explosion effect and applies external forces to nearby dynamic entities too.

* Robot: Robots are derived from Dynamic Object and they have 500 hp and 100 fuel. Their patrol area is between 220-330 meters. They have to sensor fixtures implemented with box2d. first one senses nearby robots and the second one senses incoming robots. In this class, Interception, Swarm, Rocket chasing algorithms are applied using forces with box2d physics engine.

* Rocket: Rockets are Dynamic Objects which are fired from bases and they deal 100 damage on impact to entities and explode. Rocket class also has a function called handleDrag() which introduces drag forces on it to make its move more realistic.

* Engine: Engine is the class which opens up main SFML window and runs the simulation in a loop. Every entities act() function is called each timeStep. Engine also reads mainSettings.config file for initial configurations like window size or scale. It also handles dead entities at the end of each time step for memory efficiency.

* ObjectFactory: Object factory class is implemented for producing simulation entities faster and easier.

* SensorContactListener: This class is derived from b2ContactListener class in box2d library. Main purpose of this class is dealing with user defined collisions and fire up events. At the simulation, every damage dealing and sensor sensing operations are handled in this class.

The scenario of this simulation includes Swarm robots try to intercept dangerous rockets incoming by estimating their trajectories. Boids algorithm has been extended in this work.Here is an example;

![alt text](https://github.com/yatiyr/SwarmBattle/blob/master/interceptionExample.gif "Logo Title Text 1")
