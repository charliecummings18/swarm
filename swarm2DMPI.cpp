#include <iostream>
#include <stdlib.h> 
#include <stdio.h>
#include <ctime>
#include <vector>
#include <array>
#include <functional>
#include "constants.h"
#include <mpi.h>
#include <tuple>
#include <cmath>
#include <fstream>
#include <string>



// Boid class. This holds the boids X,Y positons and velocities.
// The Boid function is used to set the initial positions aand velocities for the boid.
// The update function is used to update its position and velocity after each timestep.
// The get* functions are used to retrieve specific values about the boid at that moment in time.

class Boid {
        
private: 
    double m_X{};
    double m_Y{};
        
    double m_Xvel{};
    double m_Yvel{};
    
    int m_id{};
public:    
    
    Boid(double x, double y, double xVel, double yVel, int id)
        :m_X{x}, m_Y{y}, m_Xvel{xVel}, m_Yvel{yVel}, m_id{id}
    {
    }
    
    void update(double x, double y, double xVel, double yVel, int id) {
        
        m_X = x;
        m_Y = y;      
        m_Xvel = xVel;
        m_Yvel = yVel; 
        m_id = id;
    }
    
    double getX()  {return m_X;}
    double getY()  {return m_Y;}  
    double getXvel()  {return m_Xvel;}   
    double getYvel()  {return m_Yvel;}  
    int getid() const {return m_id;}
    
};

// The Flock (a list of Boid objects)
// This class holds all the boids and using the generate function, generates a set number
// of boids with random initial positions and velocities.

class Flock {   
private:
        int m_numBoids{};
    
public:
    
    std::vector<Boid> m_boids; 
    std::vector<Boid> neighbour(Boid& boid, const double visibility);
    
    void flockSize (int numBoids){
        m_numBoids = numBoids;
    }
    
    void generate(int boid_end) {
        double X, Y, Xvel, Yvel;
        int i;
        unsigned int seed = 88;
        std::srand(seed); 

            for (i = 0; i < boid_end ; i++) {
                
                X = static_cast<double>(std::rand())*2*WIDTH/RAND_MAX - WIDTH;
                Y = static_cast<double>(std::rand())*2*HEIGHT/RAND_MAX - HEIGHT;              
                Xvel = static_cast<double>(std::rand())*2*MAX_SPEED/RAND_MAX - MAX_SPEED; 
                Yvel = static_cast<double>(std::rand())*2*MAX_SPEED/RAND_MAX - MAX_SPEED;
                
                m_boids.push_back(Boid(X, Y, Xvel, Yvel, i));
            }
    }
    
     
    
    
// ALIGN RULE
// This aligns a boids velocity with the average velocity of its neighbours
// Neighbours are in the range ALIGN_VISIBILITY and the rule has strength ALIGN_FORCE

    std::tuple<double,double> align(Boid& boid){
        double tot_Xvel = 0,  tot_Yvel = 0, steering_Xvel = 0, 
              steering_Yvel = 0, desiredXvel = 0, desiredYvel = 0;


        std::vector<Boid> localBoids = neighbour(boid, ALIGN_VISIBILITY);
        
        if (boid.getid() >= PREDATORS)
        {
            for (int i = 0; i < m_numBoids; i++) {
            
                int curr_id = m_boids[i].getid();
                for (int j = 0; j < localBoids.size(); j++ ) {
                
                    if ( curr_id == localBoids[j].getid() && curr_id > PREDATORS) {
                    
                        tot_Xvel += m_boids[i].getXvel();
                        tot_Yvel += m_boids[i].getYvel();               
                    }
                
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
// This changes a boids velocity towards the average position of its neighbours
// Neighbours are in the range COHESION_VISIBILITY and the rule has strength COHESION_FORCE

    std::tuple<double,double> cohesion(Boid& boid){
        double tot_X = 0,  tot_Y = 0, steering_X = 0, 
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
// This ensures boids do not become too close to eachother and will begin to seperate 
// from boids when they are in the radius SEPERATION_VISIBILITY
// This rule has strength SEPERATION_FORCE

    std::tuple<double,double> seperation(Boid& boid){
        double X_sep = 0,  Y_sep = 0, X_repulsion = 0, Y_repulsion = 0;
        double distance;


        std::vector<Boid> localBoids = neighbour(boid, SEPERATION_VISIBILITY);
        
        if (boid.getid() >= PREDATORS){
            for (int i = 0; i < m_numBoids; i++) {
            
                int curr_id = m_boids[i].getid();
                for (int j = 0; j < localBoids.size(); j++ ) {
                
                    if ( curr_id == localBoids[j].getid() && curr_id > PREDATORS) {
                        distance = sqrt( pow((boid.getX() - m_boids[i].getX()),2.0) + pow((boid.getY() - m_boids[i].getY()),2.0));

                    
                        X_sep += ( boid.getX() - m_boids[i].getX() ) / distance;
                        Y_sep += ( boid.getY() - m_boids[i].getY() ) / distance;                    
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
        
        return std::make_tuple(X_repulsion, Y_repulsion);
    }
    
// PREDATOR
// Predators are boids which hunt down the normal boids. They only follow the cohesion rule, to align their position 
// with neighbouring boids (except other predators)
// Normal boids will change their velocity to move away from the predators which is given by the predator rule below.
// The predator rule is very similar to the seperation rule but the force does not scale with distance and comes into 
// action when predators are in the range PREDATOR VISIBILITY and has the force PREDATOR FORCE.

    std::tuple<double,double> predator(Boid& boid){
        double X_pred = 0,  Y_pred = 0, X_repulse = 0, Y_repulse = 0;
        double distance;


        std::vector<Boid> localBoids = neighbour(boid, PREDATOR_VISIBILITY);
        
        if (boid.getid() >= PREDATORS) {
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
    

// ADVANCE
// This function advances the boids one timestep every time it is called.
// It takes one boid at a time and changes its X/Y velocity according to the 4 rules (align,cohesion,seperation,predator).
// Velocities are used to change the boids position and then all 4 variables are updated in boid.update().
// There are two options for how boids behave when they reach the edges of the screen.
// Option 0: Boids steer away from the edges. They will feel a force within a zone near the edge which will push them away 
//           from the edges of the screen.
// Option 1: Boids reappear on the other side of the screen. This changes their positions so that the boids wrap around the screen.
    
    
    void advance(Boid& boid, int option) {
        


        double X = boid.getX();
        double Y = boid.getY();     
        double Xvel = 0;
        double Yvel = 0;     
        double magnitude = 0;  
        
        std::tuple<double,double> alignVel = align(boid);
        std::tuple<double,double> cohVel = cohesion(boid);
        std::tuple<double,double> sepVel = seperation(boid); 
        std::tuple<double,double> predVel = predator(boid);        
        
        Xvel = boid.getXvel() + std::get<0>(alignVel) + std::get<0>(cohVel) + std::get<0>(sepVel) + std::get<0>(predVel);
        Yvel = boid.getYvel() + std::get<1>(alignVel) + std::get<1>(cohVel) + std::get<1>(sepVel) + std::get<1>(predVel);      
        
 //      Steer Away from the edges (Option 0)       
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

//      Reappear the other side of the box (Option 1)    
        if (option == 1) {
            if (X > WIDTH || X < -WIDTH){
                X = -X;
            }
            if (Y > HEIGHT || Y < -HEIGHT){
                Y = -Y;
            }         
        }

            

            
        boid.update(X, Y, Xvel, Yvel, boid.getid());
        

       
    }      
};

// NEIGHBOUR
// The neighbour function generates a list of boids which are within a specified radius of the boid in question

std::vector<Boid> Flock :: neighbour(Boid& boid, const double visibility) {
    

    std::vector<Boid> Neighbours;
    
    for (int i =0; i < m_numBoids; i++) {
        
        double Xdist = m_boids[i].getX() - boid.getX();
        double Ydist = m_boids[i].getY() - boid.getY();      
        
        double distance = sqrt( pow(Xdist, 2.0) + pow(Ydist, 2.0 ));
        
        if (boid.getid() != m_boids[i].getid() && distance < visibility) {
            Neighbours.push_back( m_boids[i] );
        }
        
    }
    
    return Neighbours;
}

    

int main(int argc, char *argv[]) {

    /*
    if (argc < 2) {
        printf("Usage: \n ./swarm2D [number of processes]\n");
        return 1;
    }
    */
    

    int k, size, rank, err, X, Y, Xvel, Yvel;
    double wtime, CHUNKSIZE;   
    
    err = MPI_Init ( NULL, NULL );
    err = MPI_Comm_size ( MPI_COMM_WORLD, &size );
    err = MPI_Comm_rank ( MPI_COMM_WORLD, &rank );
    
    wtime = MPI_Wtime();
    
    CHUNKSIZE = (2*WIDTH)/(double(size));
    
    // Initialise data file
    std::string fileName;
    fileName = "boid_data2D.csv";
    std::ofstream data(fileName); 
    // Initialise file with generic info e.g. constants
    std::ofstream infoFile("infoFile2D.csv");
    
    // Create flock (in this case birds)
    Flock birds{};
    birds.flockSize(NUM_BOIDS);
    //    wtime = MPI_Wtime() - wtime;
    //    printf("Birds create: %8.6f s\n",wtime); 
    
    // generate birds in the flock with random positions

    birds.generate(NUM_BOIDS);
    
   
    
    
    if (rank == MASTER){    
    // Write infoFile.csv file
        infoFile << "HEIGHT, WIDTH, MAX_SPEED, TIME_LIMIT, TIME_STEP, NUM_BOIDS, ALIGN_VISIBILITY, \
                    COHESION_VISIBILITY, SEPERATION_VISIBILITY, ALIGN_FORCE,  COHESION_FORCE, SEPERATION_FORCE, PREDATORS\n";
    
        infoFile << std::to_string(HEIGHT) + "," + std::to_string(WIDTH) + "," + std::to_string(MAX_SPEED) \
                    + "," + std::to_string(TIME_LIMIT) + "," + std::to_string(TIME_STEP) + "," \
                    + std::to_string(NUM_BOIDS) + "," + std::to_string(ALIGN_VISIBILITY) + "," \
                    + std::to_string(COHESION_VISIBILITY) + "," + std::to_string(SEPERATION_VISIBILITY)\
                    + "," + std::to_string(ALIGN_FORCE) +"," + std::to_string(COHESION_FORCE) + ","\
                    + std::to_string(SEPERATION_FORCE) + ","\
                    +std::to_string(PREDATORS);
                

    // Write header columns for boid_data2D.csv file
    
        for (int num = 0; num < NUM_BOIDS; num ++) 
        {
            data << "boid" + std::to_string(num) + ".x, boid" 
                           + std::to_string(num) + ".y, ";
        }
        data << '\n';
    }
    
    if (rank == MASTER){
            wtime = MPI_Wtime() - wtime;
            printf("before advancement: %f\n", wtime);
        }
    // Advance the birds   
    double time = 0;            

    while (time < 0.009) {
        
        int boid_count;

        
        double *RECV_x = (double *)malloc(NUM_BOIDS*sizeof(double));
        double *RECV_y = (double *)malloc(NUM_BOIDS*sizeof(double));
        double *RECV_xvel = (double *)malloc(NUM_BOIDS*sizeof(double));
        double *RECV_yvel = (double *)malloc(NUM_BOIDS*sizeof(double));
        int    *RECV_id = (int *)malloc(NUM_BOIDS*sizeof(int));

        
        int *worker_sizes = (int *)malloc(size*sizeof(int));
        int displs[size];
        displs[0]=0;
        
        MPI_Request Request;            
        MPI_Status Status;
        int* MASTER_boid_id_set;
        
/////////////////// MASTER PROCESS /////////////////////////
        if (rank == MASTER) {
            for (int num = 0; num < NUM_BOIDS; num ++) {
                data << birds.m_boids[num].getX() << "," 
                     << birds.m_boids[num].getY() <<","; 
            }
            data << '\n';
        }
        // Each process updates chunk of boids dependent on x position
       
        std::vector<int>temp_id;

        for (k = 0; k < NUM_BOIDS; k++) {
                
            if (rank == MASTER && birds.m_boids[k].getX() < (-WIDTH + CHUNKSIZE*(rank+1)) ) {
; 
                    temp_id.push_back(birds.m_boids[k].getid());
            }
                
            else if (rank == (size-1) && birds.m_boids[k].getX() > (-WIDTH + CHUNKSIZE*rank) ) {
                    temp_id.push_back(birds.m_boids[k].getid());
            }
                
            else if (birds.m_boids[k].getX() < (-WIDTH + CHUNKSIZE*(rank+1)) && birds.m_boids[k].getX() > (-WIDTH + CHUNKSIZE*rank) ) {
                    temp_id.push_back(birds.m_boids[k].getid());
            }
        }
                
        boid_count = temp_id.size();
        if (rank == MASTER){
            wtime = MPI_Wtime() - wtime;
            printf("allocation: %f\n", wtime);
        }
        double *curr_x = (double *)calloc(boid_count,sizeof(double));
        double *curr_y = (double *)calloc(boid_count,sizeof(double));
        double *curr_xvel = (double *)calloc(boid_count,sizeof(double));
        double *curr_yvel = (double *)calloc(boid_count,sizeof(double));
        int *curr_id = (int *)calloc(boid_count,sizeof(int));
            
        // ADVANCE BOIDS BY ONE TIMESTEP
        for (int i = 0; i < boid_count; i ++) {
                birds.advance(birds.m_boids[temp_id[i]], OPTION);
                curr_x[i] = birds.m_boids[temp_id[i]].getX();
                curr_y[i] = birds.m_boids[temp_id[i]].getY();
                curr_xvel[i] = birds.m_boids[temp_id[i]].getXvel();
                curr_yvel[i] = birds.m_boids[temp_id[i]].getYvel();
                curr_id[i] = birds.m_boids[temp_id[i]].getid();
        }
        if (rank == MASTER){
            wtime = MPI_Wtime() - wtime;
            printf("advanced: %f\n", wtime);
        }
        
        

        // ALLGATHER WORKER SIZES AND DISPLACEMENTS
        
        
        err = MPI_Allgather(&boid_count, 1, MPI_INT, worker_sizes, 1, MPI_INT, MPI_COMM_WORLD);
        
        
        displs[MASTER] = 0;
        for (int i=1 ; i < size; i++){
            displs[i] = worker_sizes[i-1] + displs[i-1];
        }

        
        
        // ALLGATHER ALL BOID DATA
        
        

        err = MPI_Allgatherv(curr_x, boid_count, MPI_DOUBLE, RECV_x, worker_sizes,displs ,MPI_DOUBLE, MPI_COMM_WORLD); 
        err = MPI_Allgatherv(curr_y, boid_count, MPI_DOUBLE, RECV_y, worker_sizes,displs ,MPI_DOUBLE, MPI_COMM_WORLD); 
        err = MPI_Allgatherv(curr_xvel, boid_count, MPI_DOUBLE, RECV_xvel, worker_sizes, displs, MPI_DOUBLE, MPI_COMM_WORLD);         
        err = MPI_Allgatherv(curr_yvel, boid_count, MPI_DOUBLE, RECV_yvel, worker_sizes, displs, MPI_DOUBLE, MPI_COMM_WORLD); 
        err = MPI_Allgatherv(curr_id, boid_count, MPI_INT, RECV_id, worker_sizes, displs, MPI_INT, MPI_COMM_WORLD);       
        if (rank == MASTER){
            wtime = MPI_Wtime() - wtime;
            printf("all gather: %f\n", wtime);
        }

        for (int j = 0; j < NUM_BOIDS; j++){
            
            int id = RECV_id[j];

            birds.m_boids[id].update(RECV_x[j], RECV_y[j] , RECV_xvel[j] , RECV_yvel[j] , RECV_id[j] ); 
                 
            }
            
        if (rank == MASTER){
            wtime = MPI_Wtime() - wtime;
            printf("updated: %f\n", wtime);
        }
  
  
    temp_id.clear();
    
    
    time += TIME_STEP;
    }


    
    data.close();
    
    double total_time = MPI_Wtime() - wtime;       
    if (rank == MASTER) {
        printf("Total Elapsed-C++: %8.6f s\n",total_time); 
    }
    
    MPI_Finalize();
    
    return 0;
    
}
