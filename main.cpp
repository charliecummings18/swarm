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
    Boid() : m_X{WIDTH/2},
             m_Y{HEIGHT/2},
             m_Xvel{}, 
             m_Yvel{}
    {
    }
    
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
    std::vector<Boid> neighbour(Boid& boid);
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

                m_boids.push_back(Boid(X,Y,Xvel,Yvel, i));
            }
    }
    
    
    
// ALIGN    
    std::pair<float,float> align(Boid& boid){
        float tot_Xvel, tot_Yvel, steering_Xvel, steering_Yvel;
        
        std::vector<Boid> localBoids = neighbour(boid);
        
        for (int i = 0; i < m_numBoids; i++) {
            
//           int curr_id = m_boids[i].getid();
//            for (int j = 0; j < localBoids.size(); j++ ) {
                
 //               if ( curr_id == localBoids[j].getid() ) {
                    
                    tot_Xvel += m_boids[i].getXvel();
                    tot_Yvel += m_boids[i].getYvel(); 
//                }
                
 //           }
                    
        }
        steering_Xvel = (tot_Xvel / m_numBoids) - boid.getXvel();
        steering_Yvel = (tot_Yvel / m_numBoids) - boid.getYvel();
        
        return std::make_pair(steering_Xvel, steering_Yvel);
    }    
//


    void advance(Boid& boid) {
    
        float dt = 0.1;
        float t = 0;
        float timeLimit = 2;
        float X = boid.getX();
        float Y = boid.getY();
        float Xvel, Yvel;
//      std::ofstream data("boid_data.txt"); 
//      data << "Bird Number: " << boid.getid() << "\n\n";
//      #pragma omp parallel reduction (+:X, Y, t)
        {
        while (t < timeLimit) {
            std::pair<float, float> alignVel= align(boid);
            Xvel = alignVel.first;
            Yvel = alignVel.second;
            


            
            X += Xvel * dt;
            Y += Yvel * dt;

            std::cout<< X << " " << Y <<'\n';
//          data << X << " " << Y <<'\n';
            
            boid.update(X, Y, Xvel, Yvel);
            t+=0.1;
            }
        }
//    data.close();
    }      
};

std::vector<Boid> Flock :: neighbour(Boid& boid) {
    
    float visibility = 10;
    std::vector<Boid> Neighbours;
    
    for (int i =0; i < m_numBoids; i++) {
        
        float Xdist = m_boids[i].getX() - boid.getX();
        float Ydist = m_boids[i].getY() - boid.getY();
        
        float distance = sqrt( pow(Xdist, 2.0) + pow(Ydist, 2.0 ) );
        
        if (boid.getid() != m_boids[i].getid() && distance < visibility) {
            Neighbours.push_back( m_boids[i] );
        }
        
    }
    return Neighbours;
}

    

int main(int argc, char *argv[]) {

    if (argc < 2) {
        printf("Usage: \n ./array [number of threads]\n");
        return 1;
    }
    
    
    int threads = atoi(argv[1]);
    omp_set_num_threads( threads );
    int numBirds{10}, k;
    double initial, final, t1, t2, t3;   
    
    initial = omp_get_wtime();
    
// Create birds flock
    Flock birds{};
    birds.flockSize(numBirds);
    t1 = omp_get_wtime();
    printf("Birds create: %8.6f s\n",t1-initial); 
    
// Generate the flock of birds    
    birds.generate();
    t2 = omp_get_wtime();
    printf("Birds generate: %8.6f s\n",t2-initial);    
    
    
    for (int i = 0; i < numBirds ; i++) {
        std::cout<< "Bird no:"<< i <<
                    "   X: "<< birds.m_boids[i].getX() << 
                    " Y: "<< birds.m_boids[i].getY() <<
                    " Xvel: "<< birds.m_boids[i].getXvel() <<
                    " Yvel: "<< birds.m_boids[i].getYvel() <<'\n';
    }  
    
// Advance the birds  
    #pragma omp parallel private(k)
    {
        #pragma omp for
        for (k = 0; k < numBirds; k++) {
            std::cout<<"Bird no: "<<k<<'\n';
            birds.advance(birds.m_boids[k]);
        }
    }
    
    
        for (int i = 0; i < numBirds ; i++) {
        std::cout<< "Bird no:"<< i <<
                    "   X: "<< birds.m_boids[i].getX() << 
                    " Y: "<< birds.m_boids[i].getY() <<
                    " Xvel: "<< birds.m_boids[i].getXvel() <<
                    " Yvel: "<< birds.m_boids[i].getYvel() <<'\n';
        }
    t3 = omp_get_wtime();
    printf("Birds advance: %8.6f s\n",t3-initial);        
   
    
    final = omp_get_wtime();
    printf("Total Elapsed %8.6f s\n",final-initial); 
    

    
    /*
    for (int i = 0; i < 1 ; i++) {
        std::cout<< "Bird no:"<< i <<
                    "   X: "<< birds.m_boids[i].getX() << 
                    " Y: "<< birds.m_boids[i].getY() <<
                    " Xvel: "<< birds.m_boids[i].getXvel() <<
                    " Yvel: "<< birds.m_boids[i].getYvel() <<'\n';
    }   
*/
    
    
    return 0;
    
}
