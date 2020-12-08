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
        float tot_Xvel = 0;
        float tot_Yvel = 0;
        float steering_Xvel = 0;
        float steering_Yvel = 0;
        
        std::vector<Boid> localBoids = neighbour(boid);
        

        for (int i = 0; i < m_numBoids; i++) {
            
           int curr_id = m_boids[i].getid();
            for (int j = 0; j < localBoids.size(); j++ ) {
                
                if ( curr_id == localBoids[j].getid() ) {
                    
                    tot_Xvel += m_boids[i].getXvel();
                    tot_Yvel += m_boids[i].getYvel(); 
                }
                
            }
                    
        }
        float desiredXvel = (tot_Xvel / localBoids.size());
        float desiredYvel = (tot_Yvel / localBoids.size());
        
        if (desiredXvel != 0)
            steering_Xvel = desiredXvel - boid.getXvel();
        if (desiredYvel != 0)
            steering_Yvel = desiredYvel - boid.getYvel();
        
        return std::make_pair(steering_Xvel,steering_Yvel);
    }    



    void advance(Boid& boid, std::ofstream& file) {
        


        float X = boid.getX();
        float Y = boid.getY();
        float Xvel = boid.getXvel();
        float Yvel = boid.getYvel();
        float alignXvel, alignYvel;
        
        std::pair<float,float> alignVel = align(boid);
        if (boid.getid() == 1) {
            printf("AlignXvel: %f, AlignYvel: %f, Xvel: %f, Yvel: %f\n",alignVel.first, alignVel.second, Xvel, Yvel);
        }
        Xvel += alignVel.first;
        Yvel += alignVel.second;
        
        
        X += Xvel * TIME_STEP;
        Y += Yvel * TIME_STEP;

           

            
        boid.update(X, Y, Xvel, Yvel);
        

       
    }      
};

std::vector<Boid> Flock :: neighbour(Boid& boid) {
    

    std::vector<Boid> Neighbours;
    
    for (int i =0; i < m_numBoids; i++) {
        
        float Xdist = m_boids[i].getX() - boid.getX();
        float Ydist = m_boids[i].getY() - boid.getY();
        
        float distance = sqrt( pow(Xdist, 2.0) + pow(Ydist, 2.0 ) );
        
        if (boid.getid() != m_boids[i].getid() && distance < VISIBILITY) {
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
    int numBirds{NUM_BOIDS}, k;
    double initial, final, t1, t2, t3;   
    
    initial = omp_get_wtime();
// Initialise data file
    std::string fileName;
    fileName = "boid_data.csv";
    std::ofstream data(fileName); 
    
// Create birds flock
    Flock birds{};
    birds.flockSize(numBirds);
    t1 = omp_get_wtime();
    printf("Birds create: %8.6f s\n",t1-initial); 
    
// Generate the flock of birds    
    birds.generate();
    t2 = omp_get_wtime();
    printf("Birds generate: %8.6f s\n",t2-initial);    
 
    
// Advance the birds   
    
    
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
                     << birds.m_boids[num].getY() <<"," ;
                
            }
            data << '\n';
        }
        
    #pragma omp parallel private(k)
    {
        #pragma omp for
        for (k = 0; k < numBirds; k++) {
            birds.advance(birds.m_boids[k], data);
        }
    }
    time += TIME_STEP;
    }
    
    data.close();
    
    
    
    
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
