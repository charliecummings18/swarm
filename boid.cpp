#include "boid.h"

    
Boid::Boid(double x, double y, double xVel, double yVel, int id)
    :m_X{x}, m_Y{y}, m_Xvel{xVel}, m_Yvel{yVel}, m_id{id}
{
}
    
void Boid::update(double x, double y, double xVel, double yVel, int id) {
        
    m_X = x;
    m_Y = y;      
    m_Xvel = xVel;
    m_Yvel = yVel; 
    m_id = id;
}
    
double Boid::getX()  {return m_X;}
double Boid::getY()  {return m_Y;}  
double Boid::getXvel()  {return m_Xvel;}   
double Boid::getYvel()  {return m_Yvel;}  
int  Boid::getid() {return m_id;}
    

