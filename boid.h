#ifndef BOID_H
#define BOID_H


// Boid class. This holds the boids X,Y positons and velocities.
// The Boid function is used to set the initial positions aand velocities for the boid.
// The update function is used to update its position and velocity after each timestep.
// The get* functions are used to retrieve specific values about the boid at that moment in time.

class Boid {
        
    double m_X{};
    double m_Y{};
        
    double m_Xvel{};
    double m_Yvel{};
    
    int m_id{};
    
public:    
    
    Boid(double x, double y, double xVel, double yVel, int id);
    void update(double x, double y, double xVel, double yVel, int id); 
    double getX();  
    double getY();   
    double getXvel();  
    double getYvel();  
    int getid(); 
    
};

#endif
