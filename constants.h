#ifndef CONSTANTS_H
#define CONTANTS_H


const double HEIGHT = 180.0;
const double WIDTH = 320.0;
const double DEPTH = 180.0;

const double MAX_SPEED = 100.0;

const double TIME_LIMIT = 10;
const double TIME_STEP = 0.01;


const int NUM_BOIDS = 20;
const double ALIGN_VISIBILITY = 40;
const double COHESION_VISIBILITY = 80;
const double SEPERATION_VISIBILITY = 40;
const double PREDATOR_VISIBILITY = 60;

const double ALIGN_FORCE = 0.1;
const double COHESION_FORCE = 0.1;
const double SEPERATION_FORCE = 0.5;
const double PREDATOR_FORCE = 0.5;

const double BUFFER_ZONE = 5;
const double TURN_FORCE = 5;

const int PREDATORS = 8;
const int OPTION = 0;

const int MASTER = 0;
#endif
