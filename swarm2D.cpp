#include <iostream>
#include <stdlib.h> 
#include <stdio.h>
#include <ctime>
#include <vector>
#include <functional>
#include "constants.h"
#include <omp.h>
#include <tuple>
#include <cmath>
#include <fstream>
#include <string>




class Boid {
        
private: 
    float m_X{};
    float m_Y{};
        
    float m_Xvel{};
    float m_Yvel{};
    
    int m_id{};
public:    
    
    Boid(float x, float y, float xVel, float yVel, int id)
        :m_X{x}, m_Y{y}, m_Xvel{xVel}, m_Yvel{yVel}, m_id{id}
    {
    }
    
    void update(float x, float y, float xVel, float yVel) {
        
        m_X = x;
        m_Y = y;      
        m_Xvel = xVel;
        m_Yvel = yVel;      
    }
    
    float getX()  {return m_X;}
    float getY()  {return m_Y;}  
    float getXvel()  {return m_Xvel;}   
    float getYvel()  {return m_Yvel;}  
    int getid() const {return m_id;}
    
};

// The Flock (a list of Boid objects)

class Flock {   
private:
        int m_numBoids{};
    
public:
    
    std::vector<Boid> m_boids; 
    std::vector<Boid> neighbour(Boid& boid, const float visibility);
    void flockSize (int numBoids){
        m_numBoids = numBoids;
    }
    
    void generate() {
        float X, Y, Xvel, Yvel;
        int i;
        unsigned int seed = 88;
        std::srand(seed); 

            for (i = 0; i < m_numBoids ; i++) {
                
                X = static_cast<float>(std::rand())*2*WIDTH/RAND_MAX - WIDTH;
                Y = static_cast<float>(std::rand())*2*HEIGHT/RAND_MAX - HEIGHT;              
                Xvel = static_cast<float>(std::rand())*2*MAX_SPEED/RAND_MAX - MAX_SPEED; 
                Yvel = static_cast<float>(std::rand())*2*MAX_SPEED/RAND_MAX - MAX_SPEED;
                
                m_boids.push_back(Boid(X, Y, Xvel, Yvel, i));
            }
    }
    
    
    
// ALIGN    
    std::tuple<float,float> align(Boid& boid){
        float tot_Xvel = 0,  tot_Yvel = 0, steering_Xvel = 0, 
              steering_Yvel = 0, desiredXvel = 0, desiredYvel = 0;


        std::vector<Boid> localBoids = neighbour(boid, ALIGN_VISIBILITY);
        

        for (int i = 0; i < m_numBoids; i++) {
            
           int curr_id = m_boids[i].getid();
            for (int j = 0; j < localBoids.size(); j++ ) {
                
                if ( curr_id == localBoids[j].getid() && curr_id > PREDATORS) {
                    
                    tot_Xvel += m_boids[i].getXvel();
                    tot_Yvel += m_boids[i].getYvel();               
                }
                
            }
                    
        }
        if (localBoids.size() !=0)
        {
            desiredXvel = (tot_Xvel / localBoids.size());
            desiredYvel = (tot_Yvel / localBoids.size());         
        }
        
        if (desiredXvel != 0)
            steering_Xvel = (desiredXvel - boid.getXvel()) * ALIGN_FORCE;
        if (desiredYvel != 0)
            steering_Yvel = (desiredYvel - boid.getYvel()) * ALIGN_FORCE;   

        return std::make_tuple(steering_Xvel,steering_Yvel);
    } 
    
// COHESION    
    std::tuple<float,float> cohesion(Boid& boid){
        float tot_X = 0,  tot_Y = 0, steering_X = 0, 
              steering_Y = 0, desiredX = 0, desiredY = 0;


        std::vector<Boid> localBoids = neighbour(boid, COHESION_VISIBILITY);
        

        for (int i = 0; i < m_numBoids; i++) {
            
           int curr_id = m_boids[i].getid();
            for (int j = 0; j < localBoids.size(); j++ ) {
                
                if ( curr_id == localBoids[j].getid() && curr_id > PREDATORS) {
                    
                    tot_X += m_boids[i].getX();
                    tot_Y += m_boids[i].getY();                   
                }
                
            }
                    
        }
        if (localBoids.size() !=0)
        {
            desiredX = (tot_X / localBoids.size());
            desiredY = (tot_Y / localBoids.size());         
        }
        
        if (desiredX != 0)
            steering_X = (desiredX - boid.getX()) * COHESION_FORCE;
        if (desiredY != 0)
            steering_Y = (desiredY - boid.getY()) * COHESION_FORCE;       

        return std::make_tuple(steering_X,steering_Y);
    }     

// SEPERATION   
    std::tuple<float,float> seperation(Boid& boid){
        float X_sep = 0,  Y_sep = 0, X_repulsion = 0, Y_repulsion = 0;
        float distance;


        std::vector<Boid> localBoids = neighbour(boid, SEPERATION_VISIBILITY);
        

        for (int i = 0; i < m_numBoids; i++) {
            
           int curr_id = m_boids[i].getid();
            for (int j = 0; j < localBoids.size(); j++ ) {
                
                if ( curr_id == localBoids[j].getid() && curr_id > PREDATORS) {
                    distance = sqrt( pow((boid.getX() - m_boids[i].getX()),2.0) + pow((boid.getY() - m_boids[i].getY()),2.0));

                    
                    X_sep += ( boid.getX() - m_boids[i].getX() ) / pow(distance,1.0);
                    Y_sep += ( boid.getY() - m_boids[i].getY() ) / pow(distance,1.0);                    
                }
                
            }
        }

        if (localBoids.size() !=0 && X_sep != 0){ 
            X_repulsion = (X_sep - boid.getXvel()) * SEPERATION_FORCE;
        }
        if (localBoids.size() !=0 && Y_sep != 0){            
            Y_repulsion = (Y_sep - boid.getYvel()) * SEPERATION_FORCE;
        }     
        
        return std::make_tuple(X_repulsion, Y_repulsion);
    }
    
// PREDATOR
    std::tuple<float,float> predator(Boid& boid){
        float X_pred = 0,  Y_pred = 0, X_repulse = 0, Y_repulse = 0;
        float distance;


        std::vector<Boid> localBoids = neighbour(boid, PREDATOR_VISIBILITY);
        
        if (boid.getid() > PREDATORS) {
            for (int i = 0; i < m_numBoids; i++) {
            
                int curr_id = m_boids[i].getid();
                for (int j = 0; j < localBoids.size(); j++ ) {
                
                    if ( curr_id == localBoids[j].getid() && curr_id < PREDATORS) {
                    
                        distance = sqrt( pow((boid.getX() - m_boids[i].getX()),2.0) + pow((boid.getY() - m_boids[i].getY()),2.0));

                    
                        X_pred += ( boid.getX() - m_boids[i].getX() );
                        Y_pred += ( boid.getY() - m_boids[i].getY() );                  
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
        
        return std::make_tuple(X_repulse, Y_repulse);
    }
    


    void advance(Boid& boid, std::ofstream& file, int option) {
        


        float X = boid.getX();
        float Y = boid.getY();     
        float Xvel = 0;
        float Yvel = 0;     
        float magnitude = 0;  
        
        std::tuple<float,float> alignVel = align(boid);
        std::tuple<float,float> cohVel = cohesion(boid);
        std::tuple<float,float> sepVel = seperation(boid); 
        std::tuple<float,float> predVel = predator(boid);        
        
        Xvel = boid.getXvel() + std::get<0>(alignVel) + std::get<0>(cohVel) + std::get<0>(sepVel) + std::get<0>(predVel);
        Yvel = boid.getYvel() + std::get<1>(alignVel) + std::get<1>(cohVel) + std::get<1>(sepVel) + std::get<1>(predVel);      
        
 //      Steer Away from the edges (Option 1)       
        if (option == 0) {
            
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
        }
        
        magnitude = sqrt( pow(Xvel,2.0) + pow(Yvel,2.0));
        
        if (magnitude != 0)
        {
        Xvel = Xvel * (MAX_SPEED / magnitude);
        Yvel = Yvel * (MAX_SPEED / magnitude);   
        }
        
        
        X += Xvel * TIME_STEP;
        Y += Yvel * TIME_STEP;      

//      Reappear the other side of the box (Option 2)    
        if (option == 1) {
            if (X > WIDTH || X < -WIDTH){
                X = -X;
            }
            if (Y > HEIGHT || Y < -HEIGHT){
                Y = -Y;
            }         
        }

            

            
        boid.update(X, Y, Xvel, Yvel);
        

       
    }      
};

std::vector<Boid> Flock :: neighbour(Boid& boid, const float visibility) {
    

    std::vector<Boid> Neighbours;
    
    for (int i =0; i < m_numBoids; i++) {
        
        float Xdist = m_boids[i].getX() - boid.getX();
        float Ydist = m_boids[i].getY() - boid.getY();      
        
        float distance = sqrt( pow(Xdist, 2.0) + pow(Ydist, 2.0 ));
        
        if (boid.getid() != m_boids[i].getid() && distance < visibility) {
            Neighbours.push_back( m_boids[i] );
        }
        
    }
    
    return Neighbours;
}

    

int main(int argc, char *argv[]) {

    if (argc < 2) {
        printf("Usage: \n ./main [number of threads]\n");
        return 1;
    }
    
    
    int threads = atoi(argv[1]);
    omp_set_num_threads( threads );
    int numBirds{NUM_BOIDS}, k;
    double initial, final, t1, t2, t3;   
    
    initial = omp_get_wtime();
// Initialise data file
    std::string fileName;
    fileName = "boid_data2D.csv";
    std::ofstream data(fileName); 
// Initialise file with number of frames
    std::ofstream infoFile("infoFile2D.csv");
    
// Create birds flock
    Flock birds{};
    birds.flockSize(numBirds);
//    t1 = omp_get_wtime();
//    printf("Birds create: %8.6f s\n",t1-initial); 
    
// Generate the flock of birds    
    birds.generate();
//    t2 = omp_get_wtime();
//    printf("Birds generate: %8.6f s\n",t2-initial);    
 
    
// Advance the birds   
    
    infoFile << "HEIGHT, WIDTH, MAX_SPEED, TIME_LIMIT, TIME_STEP, NUM_BOIDS, ALIGN_VISIBILITY, COHESION_VISIBILITY, SEPERATION_VISIBILITY, ALIGN_FORCE,  COHESION_FORCE, SEPERATION_FORCE\n";
    
    infoFile << std::to_string(HEIGHT) + "," + std::to_string(WIDTH) + "," + std::to_string(MAX_SPEED) + "," + std::to_string(TIME_LIMIT) + "," + std::to_string(TIME_STEP) + "," +  std::to_string(NUM_BOIDS) + "," + std::to_string(ALIGN_VISIBILITY) + "," + std::to_string(COHESION_VISIBILITY) + "," + std::to_string(SEPERATION_VISIBILITY) + "," + std::to_string(ALIGN_FORCE) +"," + std::to_string(COHESION_FORCE) + "," + std::to_string(SEPERATION_FORCE);
    
    
    if (omp_get_thread_num() == 0) {
        for (int num = 0; num < NUM_BOIDS; num ++) 
        {
            data << "boid" + std::to_string(num) + ".x, boid" 
                           + std::to_string(num) + ".y, ";
        }
        data << '\n';
    }

    
    float time = 0;
  
    while (time < TIME_LIMIT) {
        if (omp_get_thread_num() == 0) {
            for (int num = 0; num < NUM_BOIDS; num ++) {
                data << birds.m_boids[num].getX() << "," 
                     << birds.m_boids[num].getY() <<",";
   
                
            }
            data << '\n';
        }
        
    #pragma omp parallel private(k)
    {
        #pragma omp for
        for (k = 0; k < numBirds; k++) {
            birds.advance(birds.m_boids[k], data, OPTION);
        }
    }
    time += TIME_STEP;
    }
    
    data.close();
    
    
    final = omp_get_wtime();
    printf("Total Elapsed %8.6f s\n",final-initial); 
    
    
    
    return 0;
    
}
