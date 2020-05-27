cmake_minimum_required(VERSION 3.1)

if(${CMAKE_VERSION} VERSION_LESS 3.15)
	cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})
else()
	cmake_policy(VERSION 3.15)
endif()

project(SwarmBattle VERSION 1.0
						DESCRIPTION "This project is developed under context of Ceng580 Multi Agent Systems Course"
						LANGUAGES CXX)

set(CMAKE_CXX_FLAGS "-O3")

add_library(box2D STATIC IMPORTED)
add_library(sfmlAudio STATIC IMPORTED)
add_library(sfmlGraphics STATIC IMPORTED)
add_library(sfmlMain STATIC IMPORTED)
add_library(sfmlNetwork STATIC IMPORTED)
add_library(sfmlSystem STATIC IMPORTED)
add_library(sfmlWindow STATIC IMPORTED)

set_target_properties(box2D PROPERTIES 
	IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/libs/libbox2d.a"
	INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/includes/box2d"
)

set_target_properties(sfmlAudio PROPERTIES
	IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/libs/libsfml-audio.a"
	INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/includes/SFML"
)

set_target_properties(sfmlGraphics PROPERTIES
	IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/libs/libsfml-graphics.a"
	INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/includes/SFML"
)

set_target_properties(sfmlMain PROPERTIES
	IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/libs/libsfml-main.a"
	INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/includes/SFML"
)

set_target_properties(sfmlNetwork PROPERTIES
	IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/libs/libsfml-network.a"
	INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/includes/SFML"
)

set_target_properties(sfmlSystem PROPERTIES
	IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/libs/libsfml-system.a"
	INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/includes/SFML"
)

set_target_properties(sfmlWindow PROPERTIES
	IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/libs/libsfml-window.a"
	INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/includes/SFML"
)


set(SOURCES "${CMAKE_SOURCE_DIR}/src/main.cpp"
			"${CMAKE_SOURCE_DIR}/src/Engine.cpp"
			"${CMAKE_SOURCE_DIR}/src/ObjectFactory.cpp"
			"${CMAKE_SOURCE_DIR}/src/include/DynamicObject.h"
			"${CMAKE_SOURCE_DIR}/src/include/Engine.h"
			"${CMAKE_SOURCE_DIR}/src/include/Object.h"
			"${CMAKE_SOURCE_DIR}/src/include/ObjectEnums.h"
			"${CMAKE_SOURCE_DIR}/src/include/ObjectFactory.h"
			"${CMAKE_SOURCE_DIR}/src/include/Projectile.h"
			"${CMAKE_SOURCE_DIR}/src/include/Robot.h"
			"${CMAKE_SOURCE_DIR}/src/include/Rocket.h"
			"${CMAKE_SOURCE_DIR}/src/include/Particle.h"
			"${CMAKE_SOURCE_DIR}/src/include/SensorContactListener.h"
			"${CMAKE_SOURCE_DIR}/src/include/StaticObject.h"
			"${CMAKE_SOURCE_DIR}/src/include/Base.h"
)

add_executable(simulator "${SOURCES}")
target_include_directories(simulator PRIVATE "${CMAKE_SOURCE_DIR}/src/include")
target_link_libraries(simulator box2D 
						 sfmlAudio 
						 sfmlGraphics 
						 sfmlMain 
						 sfmlNetwork 
						 sfmlSystem 
						 sfmlWindow)
