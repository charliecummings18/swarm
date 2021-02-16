#ifndef FLOCK_H
#define FLOCK_H


// The Flock (a list of Boid objects)
// This class holds all the boids and using the generate function, generates a set number
// of boids with random initial positions and velocities.

class Flock {   
private:
        int m_numBoids{};
    
public:
    
    std::vector<Boid> m_boids; 

    std::vector<Boid> m_curr_boids;
    
    void flockSize (int numBoids);
    
    void generate(int numBoids); 
    
    void curr_boids(double X, double Y, double Xvel, double Yvel, int id);
    
    
     
    
    
// ALIGN RULE
// This aligns a boids velocity with the average velocity of its neighbours
// Neighbours are in the range ALIGN_VISIBILITY and the rule has strength ALIGN_FORCE

    std::tuple<double,double> align(Boid& boid);
    
// COHESION 
// This changes a boids velocity towards the average position of its neighbours
// Neighbours are in the range COHESION_VISIBILITY and the rule has strength COHESION_FORCE

    std::tuple<double,double> cohesion(Boid& boid);

// SEPERATION   
// This ensures boids do not become too close to eachother and will begin to seperate 
// from boids when they are in the radius SEPERATION_VISIBILITY
// This rule has strength SEPERATION_FORCE

    std::tuple<double,double> seperation(Boid& boid);
    
// PREDATOR
// Predators are boids which hunt down the normal boids. They only follow the cohesion rule, to align their position 
// with neighbouring boids (except other predators)
// Normal boids will change their velocity to move away from the predators which is given by the predator rule below.
// The predator rule is very similar to the seperation rule but the force does not scale with distance and comes into 
// action when predators are in the range PREDATOR VISIBILITY and has the force PREDATOR FORCE.

    std::tuple<double,double> predator(Boid& boid);
    
    
// NEIGHBOUR
// The neighbour function generates a list of boids which are within a specified radius of
// the boid in question
    
    std::vector<Boid> neighbour(Boid& boid, const double visibility);
    

// ADVANCE
// This function advances the boids one timestep every time it is called.
// It takes one boid at a time and changes its X/Y velocity according to the 4 rules (align,cohesion,seperation,predator).
// Velocities are used to change the boids position and then all 4 variables are updated in boid.update().
// There are two options for how boids behave when they reach the edges of the screen.
// Option 0: Boids steer away from the edges. They will feel a force within a zone near the edge which will push them away 
//           from the edges of the screen.
// Option 1: Boids reappear on the other side of the screen. This changes their positions so that the boids wrap around the screen.
    
    void advance(Boid& boid); 
    

};



#endif
