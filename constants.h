#ifndef CONSTANTS_H
#define CONTANTS_H


const float HEIGHT = 180.0;
const float WIDTH = 320.0;
const float DEPTH = 180.0;

const float MAX_SPEED = 400.0;

const float TIME_LIMIT = 10;
const float TIME_STEP = 0.01;


const int NUM_BOIDS = 100;
const float ALIGN_VISIBILITY = 40;
const float COHESION_VISIBILITY = 80;
const float SEPERATION_VISIBILITY = 40;
const float PREDATOR_VISIBILITY = 40;

const float ALIGN_FORCE = 0.1;
const float COHESION_FORCE = 0.01;
const float SEPERATION_FORCE = 0.5;
const float PREDATOR_FORCE = 10;

const float BUFFER_ZONE = 5;
const float TURN_FORCE = 20;

const int PREDATORS = 10;
const int OPTION = 0;

#endif
