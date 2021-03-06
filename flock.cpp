#include "flock.h"
void Flock :: flockSize (int numBoids){
    m_numBoids = numBoids;
}
  
  
void Flock :: generate(int numBoids, std::string method) {
    double X, Y, Z, Xvel, Yvel, Zvel;
    int i;
    unsigned int seed = 88;
    std::srand(seed); 
    if (method == "OMP"){
        #pragma omp parallel private(i, X, Y, Xvel, Yvel, Z, Zvel)
        {
            #pragma omp for 
            for (i = 0; i < numBoids ; i++) {
                
                X = static_cast<double>(std::rand())*2*WIDTH/RAND_MAX - WIDTH;
                Y = static_cast<double>(std::rand())*2*HEIGHT/RAND_MAX - HEIGHT;  
                Z = static_cast<double>(std::rand())*2*HEIGHT/RAND_MAX - DEPTH; 
                Xvel = static_cast<double>(std::rand())*2*MAX_SPEED/RAND_MAX - MAX_SPEED; 
                Yvel = static_cast<double>(std::rand())*2*MAX_SPEED/RAND_MAX - MAX_SPEED;
                Zvel = static_cast<double>(std::rand())*2*MAX_SPEED/RAND_MAX - MAX_SPEED;
                #pragma omp critical
                {
                m_curr_boids.push_back(Boid(X, Y, Xvel, Yvel, i, Z, Zvel));
                }
            }
        }
    }
    else if (method == "MPI" || method == "HYBRID"){
        for (i = 0; i < numBoids ; i++) {
                
            X = static_cast<double>(std::rand())*2*WIDTH/RAND_MAX - WIDTH;
            Y = static_cast<double>(std::rand())*2*HEIGHT/RAND_MAX - HEIGHT;  
            Z = static_cast<double>(std::rand())*2*HEIGHT/RAND_MAX - DEPTH; 
            Xvel = static_cast<double>(std::rand())*2*MAX_SPEED/RAND_MAX - MAX_SPEED; 
            Yvel = static_cast<double>(std::rand())*2*MAX_SPEED/RAND_MAX - MAX_SPEED;
            Zvel = static_cast<double>(std::rand())*2*MAX_SPEED/RAND_MAX - MAX_SPEED;
                
            m_boids.push_back(Boid(X, Y, Xvel, Yvel, i, Z, Zvel));
            }
        } 
}
    
void Flock :: curr_boids(double X, double Y, double Xvel, double Yvel, int id, double Z, double Zvel){
    m_curr_boids.push_back(Boid(X, Y, Xvel, Yvel, id, Z, Zvel));
}
    
    
     

std::tuple<double,double, double> Flock :: align(Boid& boid){
    double tot_Xvel = 0,  tot_Yvel = 0,  tot_Zvel = 0, steering_Xvel = 0, 
            steering_Yvel = 0, steering_Zvel = 0, desiredXvel = 0, desiredYvel = 0, desiredZvel = 0;


    std::vector<Boid> localBoids = neighbour(boid, ALIGN_VISIBILITY);
        
    if (boid.getid() >= PREDATORS)
    {
        for (int j = 0; j < localBoids.size(); j++ ) {
                
            if (localBoids[j].getid() > PREDATORS) {
                    
                tot_Xvel += localBoids[j].getXvel();
                tot_Yvel += localBoids[j].getYvel();
                if (DIMENSIONS == 3){
                     tot_Zvel += localBoids[j].getZvel();
                }
            }
                
        }
                    
    }
        
    if (localBoids.size() !=0)
    {
        desiredXvel = (tot_Xvel / localBoids.size());
        desiredYvel = (tot_Yvel / localBoids.size());    
        if (DIMENSIONS == 3){
            desiredZvel = (tot_Zvel / localBoids.size());    
        }
    }
        
    if (desiredXvel != 0)
        steering_Xvel = (desiredXvel - boid.getXvel()) * ALIGN_FORCE;
    if (desiredYvel != 0)
        steering_Yvel = (desiredYvel - boid.getYvel()) * ALIGN_FORCE;  
    if (desiredZvel != 0)
        steering_Zvel = (desiredZvel - boid.getZvel()) * ALIGN_FORCE;      

    return std::make_tuple(steering_Xvel,steering_Yvel, steering_Zvel);
} 
    


std::tuple<double,double,double> Flock :: cohesion(Boid& boid){
    double tot_X = 0,  tot_Y = 0, tot_Z = 0, steering_X = 0, 
            steering_Y = 0,  steering_Z = 0, desiredX = 0, desiredY = 0, desiredZ = 0;


    std::vector<Boid> localBoids = neighbour(boid, COHESION_VISIBILITY);

    for (int j = 0; j < localBoids.size(); j++ ) {
                
        if (localBoids[j].getid() > PREDATORS) {
                    
            tot_X += localBoids[j].getX();
            tot_Y += localBoids[j].getY();                     
            if (DIMENSIONS == 3){
                tot_Z += localBoids[j].getZ();
                }              
        }    
    }
    
        
    if (localBoids.size() !=0)
    {
        desiredX = (tot_X / localBoids.size());
        desiredY = (tot_Y / localBoids.size());    
        if (DIMENSIONS == 3){
            desiredZ = (tot_Z / localBoids.size()); 
        }
    }
        
    if (desiredX != 0)
        steering_X = (desiredX - boid.getX()) * COHESION_FORCE;
    if (desiredY != 0)
        steering_Y = (desiredY - boid.getY()) * COHESION_FORCE;       
    if (desiredZ != 0)
        steering_Z = (desiredZ - boid.getZ()) * COHESION_FORCE;  
    
    return std::make_tuple(steering_X,steering_Y,steering_Z);
}     


std::tuple<double,double,double> Flock :: seperation(Boid& boid){
    double X_sep = 0,  Y_sep = 0,  Z_sep = 0, X_repulsion = 0, Y_repulsion = 0, Z_repulsion = 0;
    double distance;


    std::vector<Boid> localBoids = neighbour(boid, SEPERATION_VISIBILITY);
        
    if (boid.getid() >= PREDATORS){

        for (int j = 0; j < localBoids.size(); j++ ) {
                
            if (localBoids[j].getid() > PREDATORS) {
                if (DIMENSIONS == 2){ 
                    distance = sqrt( pow((boid.getX() - localBoids[j].getX()),2.0) + pow((boid.getY() - localBoids[j].getY()),2.0));
                }
                else if (DIMENSIONS == 3){
                    distance = sqrt( pow((boid.getX() - localBoids[j].getX()),2.0) + pow((boid.getY() - localBoids[j].getY()),2.0) + pow((boid.getZ() - localBoids[j].getZ()),2.0));                    
                }

                    
                X_sep += ( boid.getX() - localBoids[j].getX() ) / distance;
                Y_sep += ( boid.getY() - localBoids[j].getY() ) / distance; 
                if (DIMENSIONS == 3){
                    Z_sep += ( boid.getZ() - localBoids[j].getZ() ) / distance; 
                }
            }
                
        }

    }

    if (localBoids.size() !=0 && X_sep != 0){ 
        X_repulsion = (X_sep - boid.getXvel()) * SEPERATION_FORCE;
    }
    if (localBoids.size() !=0 && Y_sep != 0){            
        Y_repulsion = (Y_sep - boid.getYvel()) * SEPERATION_FORCE;
    } 
    if (localBoids.size() !=0 && Z_sep != 0){            
        Z_repulsion = (Z_sep - boid.getZvel()) * SEPERATION_FORCE;
    }     
    
        
    return std::make_tuple(X_repulsion, Y_repulsion, Z_repulsion);
}
    


std::tuple<double,double,double> Flock :: predator(Boid& boid){
    double X_pred = 0,  Y_pred = 0, Z_pred = 0, X_repulse = 0, Y_repulse = 0, Z_repulse = 0;
    double distance;


    std::vector<Boid> localBoids = neighbour(boid, PREDATOR_VISIBILITY);
        
    if (boid.getid() >= PREDATORS) {

        for (int j = 0; j < localBoids.size(); j++ ) {
                
            if (localBoids[j].getid()  < PREDATORS) {
                if (DIMENSIONS == 2){
                    distance = sqrt( pow((boid.getX() - localBoids[j].getX()),2.0) + pow((boid.getY() - localBoids[j].getY()),2.0));
                }
                else if (DIMENSIONS == 3){
                    distance = sqrt( pow((boid.getX() - localBoids[j].getX()),2.0) + pow((boid.getY() - localBoids[j].getY()),2.0) + pow((boid.getZ() - localBoids[j].getZ()),2.0));                    
                }

                    
                X_pred += ( boid.getX() - localBoids[j].getX() );
                Y_pred += ( boid.getY() - localBoids[j].getY() );  
                if (DIMENSIONS == 3) {
                    Z_pred += ( boid.getZ() - localBoids[j].getZ() );    
                }
            }
                
        }
    }

    if (localBoids.size() !=0 && X_pred != 0){ 
        X_repulse = (X_pred - boid.getXvel()) * PREDATOR_FORCE;
    }
    if (localBoids.size() !=0 && Y_pred != 0){            
        Y_repulse = (Y_pred - boid.getYvel()) * PREDATOR_FORCE;
    }
     if (localBoids.size() !=0 && Z_pred != 0){            
        Z_repulse = (Z_pred - boid.getZvel()) * PREDATOR_FORCE;
    }   
    
        
    return std::make_tuple(X_repulse, Y_repulse, Z_repulse);
}
    

std::vector<Boid> Flock :: neighbour(Boid& boid, const double visibility) {

    std::vector<Boid> Neighbours;
    
    double Xdist;
    double Ydist;
    double Zdist;    
    double distance;


    for (int i = 0; i < (m_curr_boids.size()); i++) {
        
        Xdist = m_curr_boids[i].getX() - boid.getX();
        Ydist = m_curr_boids[i].getY() - boid.getY();      
        if (DIMENSIONS == 3) {
            Zdist = m_curr_boids[i].getZ() - boid.getZ();      
        }
        if (DIMENSIONS == 2){
            distance = sqrt( pow(Xdist, 2.0) + pow(Ydist, 2.0 ));
        }
        else if (DIMENSIONS == 3){
            distance = sqrt( pow(Xdist, 2.0) + pow(Ydist, 2.0 ) + pow(Zdist, 2.0 ));
        }
        if (boid.getid() != m_curr_boids[i].getid() && distance < visibility) {
            Neighbours.push_back( m_curr_boids[i] );
        }   
    }
    return Neighbours;
}    
    
void Flock :: advance(Boid& boid) {
        
    double X = boid.getX();
    double Y = boid.getY(); 
    double Z = boid.getZ();    
    double Xvel = 0;
    double Yvel = 0;  
    double Zvel = 0;    
    double magnitude = 0;  
        
    std::tuple<double,double,double> alignVel = align(boid);
    std::tuple<double,double,double> cohVel = cohesion(boid);
    std::tuple<double,double,double> sepVel = seperation(boid); 
    std::tuple<double,double,double> predVel = predator(boid);        
      
    Xvel = boid.getXvel() + std::get<0>(alignVel) + std::get<0>(cohVel) + std::get<0>(sepVel) + std::get<0>(predVel);
    Yvel = boid.getYvel() + std::get<1>(alignVel) + std::get<1>(cohVel) + std::get<1>(sepVel) + std::get<1>(predVel); 
    
    if (DIMENSIONS == 3) {
        Zvel = boid.getZvel() + std::get<2>(alignVel) + std::get<2>(cohVel) + std::get<2>(sepVel) + std::get<2>(predVel);
    }
        
        
//  Steer Away from the edges (Option 0)       
    if (OPTION == 0) {
            
        if (boid.getX() < BUFFER_ZONE - WIDTH){
            Xvel += TURN_FORCE;
        }
        if (boid.getX() > WIDTH - BUFFER_ZONE){
            Xvel -= TURN_FORCE;
        }   
        if (boid.getY() < BUFFER_ZONE - HEIGHT){
            Yvel += TURN_FORCE;
        }
        if (boid.getY() > HEIGHT - BUFFER_ZONE){
            Yvel -= TURN_FORCE;
        } 
        if (DIMENSIONS == 3){
            if (boid.getZ() < BUFFER_ZONE - DEPTH){
                Zvel += TURN_FORCE;
            }
            if (boid.getZ() > DEPTH - BUFFER_ZONE){
                Zvel -= TURN_FORCE;
            }    
        }
    }
    if (DIMENSIONS == 2){    
        magnitude = sqrt( pow(Xvel,2.0) + pow(Yvel,2.0));
    }
    else if (DIMENSIONS == 3){
        magnitude = sqrt( pow(Xvel,2.0) + pow(Yvel,2.0) + pow(Zvel,2.0));
    }
    if (magnitude != 0){
        Xvel = Xvel * (MAX_SPEED / magnitude);
        Yvel = Yvel * (MAX_SPEED / magnitude);   
        if (DIMENSIONS == 3){
            Zvel = Zvel * (MAX_SPEED / magnitude);   
        }
    }
        
        
    X += Xvel * TIME_STEP;
    Y += Yvel * TIME_STEP;    
    if (DIMENSIONS == 3){
        Z += Zvel * TIME_STEP;    
    }

//  Reappear the other side of the box (Option 1)    
    if (OPTION == 1) {
        if (X > WIDTH || X < -WIDTH){
            X = -X;
        }
        if (Y > HEIGHT || Y < -HEIGHT){
            Y = -Y;
        }
        if (DIMENSIONS == 3) {
            if (Z > DEPTH || Y < -DEPTH){
                Z = -Z;
            } 
        }
    }
  
    boid.update(X, Y, Xvel, Yvel, boid.getid(), Z, Zvel);
}
