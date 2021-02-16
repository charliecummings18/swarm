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
    std::vector<Boid> m_curr_boids;
    
    void flockSize (int numBoids){
        m_numBoids = numBoids;
    }
    
    void generate(int numBoids) {
        double X, Y, Xvel, Yvel;
        int i;
        unsigned int seed = 88;
        std::srand(seed); 

            for (i = 0; i < numBoids ; i++) {
                
                X = static_cast<double>(std::rand())*2*WIDTH/RAND_MAX - WIDTH;
                Y = static_cast<double>(std::rand())*2*HEIGHT/RAND_MAX - HEIGHT;              
                Xvel = static_cast<double>(std::rand())*2*MAX_SPEED/RAND_MAX - MAX_SPEED; 
                Yvel = static_cast<double>(std::rand())*2*MAX_SPEED/RAND_MAX - MAX_SPEED;
                
                m_boids.push_back(Boid(X, Y, Xvel, Yvel, i));
            }
    }
    
    void curr_boids(double X, double Y, double Xvel, double Yvel, int id){
        m_curr_boids.push_back(Boid(X, Y, Xvel, Yvel, id));
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
            for (int j = 0; j < localBoids.size(); j++ ) {
                
                if (localBoids[j].getid() > PREDATORS) {
                    
                    tot_Xvel += localBoids[j].getXvel();
                    tot_Yvel += localBoids[j].getYvel();               
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

        for (int j = 0; j < localBoids.size(); j++ ) {
                
            if (localBoids[j].getid() > PREDATORS) {
                    
                tot_X += localBoids[j].getX();
                tot_Y += localBoids[j].getY();                   
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

            for (int j = 0; j < localBoids.size(); j++ ) {
                
                if (localBoids[j].getid() > PREDATORS) {
                    distance = sqrt( pow((boid.getX() - localBoids[j].getX()),2.0) + pow((boid.getY() - localBoids[j].getY()),2.0));

                    
                    X_sep += ( boid.getX() - localBoids[j].getX() ) / distance;
                    Y_sep += ( boid.getY() - localBoids[j].getY() ) / distance;                    
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

            for (int j = 0; j < localBoids.size(); j++ ) {
                
                if (localBoids[j].getid()  < PREDATORS) {
                    
                    distance = sqrt( pow((boid.getX() - localBoids[j].getX()),2.0) + pow((boid.getY() - localBoids[j].getY()),2.0));

                    
                    X_pred += ( boid.getX() - localBoids[j].getX() );
                    Y_pred += ( boid.getY() - localBoids[j].getY() );                  
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
    
    
    void advance(Boid& boid) {
        
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
        if (OPTION == 1) {
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
    
    double Xdist;
    double Ydist;
    double distance;


    for (int i = 0; i < (m_curr_boids.size()); i++) {
        
        Xdist = m_curr_boids[i].getX() - boid.getX();
        Ydist = m_curr_boids[i].getY() - boid.getY();      
        
        distance = sqrt( pow(Xdist, 2.0) + pow(Ydist, 2.0 ));
        
        if (boid.getid() != m_curr_boids[i].getid() && distance < visibility) {
            Neighbours.push_back( m_curr_boids[i] );
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
    double wtime, chunksize;   
    
    err = MPI_Init ( NULL, NULL );
    err = MPI_Comm_size ( MPI_COMM_WORLD, &size );
    err = MPI_Comm_rank ( MPI_COMM_WORLD, &rank );
    int RANK = rank;
    wtime = MPI_Wtime();
    
    chunksize = (2*WIDTH)/(double(size));
    
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
    if (rank == MASTER){  
        birds.generate(NUM_BOIDS);
    
   
    
    
 
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
    
    // Advance the birds   
    double time = 0;            

    while (time < TIME_LIMIT) {
        
        int boid_count;
        
        double *curr_x = (double *)malloc(NUM_BOIDS*sizeof(double));
        double *curr_y = (double *)malloc(NUM_BOIDS*sizeof(double));
        double *curr_xvel = (double *)malloc(NUM_BOIDS*sizeof(double));
        double *curr_yvel = (double *)malloc(NUM_BOIDS*sizeof(double));
        int *curr_id = (int *)malloc(NUM_BOIDS*sizeof(int));
        
        double *MASTERcurr_x = (double *)malloc(NUM_BOIDS*sizeof(double));
        double *MASTERcurr_y = (double *)malloc(NUM_BOIDS*sizeof(double));
        double *MASTERcurr_xvel = (double *)malloc(NUM_BOIDS*sizeof(double));
        double *MASTERcurr_yvel = (double *)malloc(NUM_BOIDS*sizeof(double));
        int *MASTERcurr_id = (int *)malloc(NUM_BOIDS*sizeof(int));
        
        
        
        double *RECV_x;
        double *RECV_y;
        double *RECV_xvel;
        double *RECV_yvel;
        int    *RECV_id;
        
        
        int *worker_sizes = (int *)malloc((size)*sizeof(int));
        int *displs = (int *)malloc((size)*sizeof(int));

        MPI_Request Request;            
        MPI_Status Status;
       
/////////////////// MASTER PROCESS /////////////////////////
        if (rank == MASTER) {
            for (int num = 0; num < NUM_BOIDS; num ++) {
                data << birds.m_boids[num].getX() << "," 
                     << birds.m_boids[num].getY() <<","; 
            }
            data << '\n';
        
            // Each process updates chunk of boids dependent on x position

            worker_sizes[0]=0;
            displs[0]=0;
            displs[1]=0;
            
            int assigned_rank = 0;
            int x_chunk = -WIDTH;
           
            while (x_chunk < WIDTH) {
                int index = 0;
                for (k = 0; k < NUM_BOIDS; k++) {
                
                    if (assigned_rank == MASTER && birds.m_boids[k].getX() < (-WIDTH + chunksize*(assigned_rank+1)) ) { 
                        MASTERcurr_x[index] = birds.m_boids[k].getX();
                        MASTERcurr_y[index] = birds.m_boids[k].getY();
                        MASTERcurr_xvel[index] = birds.m_boids[k].getXvel();
                        MASTERcurr_yvel[index] = birds.m_boids[k].getYvel();
                        MASTERcurr_id[index] = birds.m_boids[k].getid();
                        index +=1;
                        
                    }
                
                    else if (assigned_rank == (size-1) && birds.m_boids[k].getX() > (-WIDTH + chunksize*assigned_rank) ) {
                        curr_x[index] = birds.m_boids[k].getX();
                        curr_y[index] = birds.m_boids[k].getY();
                        curr_xvel[index] = birds.m_boids[k].getXvel();
                        curr_yvel[index] = birds.m_boids[k].getYvel();
                        curr_id[index] = birds.m_boids[k].getid();
                        index +=1;
                    }
                
                    else if (birds.m_boids[k].getX() < (-WIDTH + chunksize*(assigned_rank+1)) && birds.m_boids[k].getX() > (-WIDTH + chunksize*assigned_rank) ) {
                        curr_x[index] = birds.m_boids[k].getX();
                        curr_y[index] = birds.m_boids[k].getY();
                        curr_xvel[index] = birds.m_boids[k].getXvel();
                        curr_yvel[index] = birds.m_boids[k].getYvel();
                        curr_id[index] = birds.m_boids[k].getid();
                        index +=1;
                    }   
                }

                if (assigned_rank != MASTER)
                {
                    worker_sizes[assigned_rank] = index;

                    displs[assigned_rank+1] = index + displs[assigned_rank];
  
        
                    err = MPI_Send(&index, 1, MPI_INT, assigned_rank, assigned_rank, MPI_COMM_WORLD);
                    
                    err = MPI_Send(curr_x, index, MPI_DOUBLE, assigned_rank, assigned_rank, MPI_COMM_WORLD);
                    err = MPI_Send(curr_y, index, MPI_DOUBLE, assigned_rank, assigned_rank, MPI_COMM_WORLD);                    
                    err = MPI_Send(curr_xvel, index, MPI_DOUBLE, assigned_rank, assigned_rank, MPI_COMM_WORLD);                    
                    err = MPI_Send(curr_yvel, index, MPI_DOUBLE, assigned_rank, assigned_rank, MPI_COMM_WORLD);                    
                    err = MPI_Send(curr_id, index, MPI_INT, assigned_rank, assigned_rank, MPI_COMM_WORLD);                    
 
                }
   
                else if (assigned_rank == MASTER){
                    boid_count = index;}
  
                assigned_rank+=1;
                x_chunk+=chunksize;
                
            } 
        }
        else if (rank != MASTER) {
            
            err = MPI_Recv(&boid_count, 1, MPI_INT, MASTER, rank, MPI_COMM_WORLD, &Status);

            err = MPI_Recv(curr_x, boid_count, MPI_DOUBLE, MASTER, rank, MPI_COMM_WORLD, &Status);
            err = MPI_Recv(curr_y, boid_count, MPI_DOUBLE, MASTER, rank, MPI_COMM_WORLD, &Status);           
            err = MPI_Recv(curr_xvel, boid_count, MPI_DOUBLE, MASTER, rank, MPI_COMM_WORLD, &Status);            
            err = MPI_Recv(curr_yvel, boid_count, MPI_DOUBLE, MASTER, rank, MPI_COMM_WORLD, &Status);            
            err = MPI_Recv(curr_id, boid_count, MPI_INT, MASTER, rank, MPI_COMM_WORLD, &Status); 
        }

        birds.m_curr_boids.clear();
        
        
        if (rank != MASTER){    
            for (int i = 0; i < boid_count; i++){
                birds.curr_boids(curr_x[i], curr_y[i], curr_xvel[i], curr_yvel[i], curr_id[i]);
            }
        }
        
        else if (rank == MASTER){    
            for (int i = 0; i < boid_count; i++){
                birds.curr_boids(MASTERcurr_x[i], MASTERcurr_y[i], MASTERcurr_xvel[i], MASTERcurr_yvel[i], MASTERcurr_id[i]);
            }
        }
    
      
           //  COMMUNICATION //     
        int left_count = 0;
        int right_count = 0;
        
        if (rank != size-1){
            
            // SENDING TO RIGHT//
            std::vector<int>temp_send_right_id;
            std::vector<std::vector<double>>temp_send_right;
            
            for (int i = 0; i<4 ; i++){
                temp_send_right.push_back(std::vector<double>());
            }
            int index = 0;
            double *send_right[4];
            int *send_right_id;
            int right_rank = rank+1;
            for (int r = 0; r < boid_count; r++) {
            
                if ( ((-WIDTH + (chunksize*(rank+1))) - birds.m_curr_boids[r].getX())   < VISIBILITY ){
                    temp_send_right[0].push_back(birds.m_curr_boids[r].getX());
                    temp_send_right[1].push_back(birds.m_curr_boids[r].getY());
                    temp_send_right[2].push_back(birds.m_curr_boids[r].getXvel());
                    temp_send_right[3].push_back(birds.m_curr_boids[r].getYvel());
                    temp_send_right_id.push_back(birds.m_curr_boids[r].getid());
                } 
            }
            int send_right_size = temp_send_right_id.size();
            send_right_id = (int *)malloc(send_right_size*sizeof(int));
            for (int i = 0; i < 4; i++){        
                send_right[i] = (double *)malloc(send_right_size * sizeof(double)); 
            }
            
            for (int i = 0; i < send_right_size; i++) {
                send_right[0][i] = temp_send_right[0][i];
                send_right[1][i] = temp_send_right[1][i];
                send_right[2][i] = temp_send_right[2][i];
                send_right[3][i] = temp_send_right[3][i];
                send_right_id[i] = temp_send_right_id[i];
            }
        
            err = MPI_Send(&send_right_size, 1, MPI_INT, right_rank , 99, MPI_COMM_WORLD);
            
            err = MPI_Send(send_right[0], send_right_size, MPI_DOUBLE, right_rank, 98, MPI_COMM_WORLD);
            err = MPI_Send(send_right[1], send_right_size, MPI_DOUBLE, right_rank, 97, MPI_COMM_WORLD);           
            err = MPI_Send(send_right[2], send_right_size, MPI_DOUBLE, right_rank, 96, MPI_COMM_WORLD);            
            err = MPI_Send(send_right[3], send_right_size, MPI_DOUBLE, right_rank, 95, MPI_COMM_WORLD);            
            
            err = MPI_Send(send_right_id, send_right_size, MPI_INT, right_rank, 94, MPI_COMM_WORLD);
        
            
            
            // RECIEVING FROM RIGHT//
            


            err = MPI_Recv(&right_count, 1, MPI_INT, right_rank, 93, MPI_COMM_WORLD, &Status);
            double *rec_right[4];
            int *rec_right_id = (int *)malloc(right_count*sizeof(int));
            for (int i = 0; i < 4; i++){        
                rec_right[i] = (double *)malloc(right_count* sizeof(double)); 
            }
            err = MPI_Recv(rec_right[0], right_count, MPI_DOUBLE, right_rank, 92, MPI_COMM_WORLD, &Status);
            err = MPI_Recv(rec_right[1], right_count, MPI_DOUBLE, right_rank, 91, MPI_COMM_WORLD, &Status);
            err = MPI_Recv(rec_right[2], right_count, MPI_DOUBLE, right_rank, 90, MPI_COMM_WORLD, &Status);
            err = MPI_Recv(rec_right[3], right_count, MPI_DOUBLE, right_rank, 89, MPI_COMM_WORLD, &Status);
            err = MPI_Recv(rec_right_id, right_count, MPI_INT, right_rank, 88, MPI_COMM_WORLD, &Status);

            for (int r = 0; r < right_count; r++) {
                birds.curr_boids(rec_right[0][r],rec_right[1][r],rec_right[2][r],rec_right[3][r],rec_right_id[r]);     
            }            
        }
            
            
        if (rank != MASTER){
            
            
            // SENDING TO LEFT//
            
            std::vector<int>temp_send_left_id;
            std::vector<std::vector<double>>temp_send_left;
            
            for (int i = 0; i<4 ; i++){
                temp_send_left.push_back(std::vector<double>());
            }
            double *send_left[4];
            int *send_left_id;
            int left_rank = rank-1;
            for (int r = 0; r < boid_count; r++) {
            
                if ( (birds.m_curr_boids[r].getX() - (-WIDTH + (chunksize*rank)))  < VISIBILITY ){
                    
                    temp_send_left[0].push_back(birds.m_curr_boids[r].getX());
                    temp_send_left[1].push_back(birds.m_curr_boids[r].getY());
                    temp_send_left[2].push_back(birds.m_curr_boids[r].getXvel());
                    temp_send_left[3].push_back(birds.m_curr_boids[r].getYvel());
                    temp_send_left_id.push_back(birds.m_curr_boids[r].getid());
                } 
            }
            int send_left_size = temp_send_left_id.size();
            send_left_id = (int *)malloc(send_left_size*sizeof(int));
            for (int i = 0; i < 4; i++){        
                send_left[i] = (double *)malloc(send_left_size* sizeof(double)); 
            }
            
            for (int i = 0; i < send_left_size; i++) {
                send_left[0][i] = temp_send_left[0][i];
                send_left[1][i] = temp_send_left[1][i];
                send_left[2][i] = temp_send_left[2][i];
                send_left[3][i] = temp_send_left[3][i];
                send_left_id[i] = temp_send_left_id[i];
            }
        
            err = MPI_Send(&send_left_size, 1, MPI_INT,left_rank , 93, MPI_COMM_WORLD);
            
            err = MPI_Send(send_left[0], send_left_size, MPI_DOUBLE, left_rank, 92, MPI_COMM_WORLD);
            err = MPI_Send(send_left[1], send_left_size, MPI_DOUBLE, left_rank, 91, MPI_COMM_WORLD);
            err = MPI_Send(send_left[2], send_left_size, MPI_DOUBLE, left_rank, 90, MPI_COMM_WORLD);
            err = MPI_Send(send_left[3], send_left_size, MPI_DOUBLE, left_rank, 89, MPI_COMM_WORLD);
            err = MPI_Send(send_left_id, send_left_size, MPI_INT, left_rank, 88, MPI_COMM_WORLD);           
            
            
            

            
            // RECIEVING FROM LEFT//

            
            
            err = MPI_Recv(&left_count, 1, MPI_INT, left_rank, 99, MPI_COMM_WORLD, &Status);
            double *rec_left[4];
            int *rec_left_id = (int *)malloc(left_count*sizeof(int));
            for (int i = 0; i < 4; i++){        
                rec_left[i] = (double *)malloc(left_count* sizeof(double)); 
            }
            err = MPI_Recv(rec_left[0], left_count, MPI_DOUBLE, left_rank, 98, MPI_COMM_WORLD, &Status);
            err = MPI_Recv(rec_left[1], left_count, MPI_DOUBLE, left_rank, 97, MPI_COMM_WORLD, &Status);
            err = MPI_Recv(rec_left[2], left_count, MPI_DOUBLE, left_rank, 96, MPI_COMM_WORLD, &Status);
            err = MPI_Recv(rec_left[3], left_count, MPI_DOUBLE, left_rank, 95, MPI_COMM_WORLD, &Status);
            err = MPI_Recv(rec_left_id, left_count, MPI_INT, left_rank, 94, MPI_COMM_WORLD, &Status);
            
            
            for (int r = 0; r < left_count; r++) {
                birds.curr_boids(rec_left[0][r],rec_left[1][r],rec_left[2][r],rec_left[3][r],rec_left_id[r]);     
            }
            
        }
        
        ///////////////////

   
    
        
        if (rank != MASTER){         
            for (int j = 0; j < boid_count; j ++) {
                birds.advance(birds.m_curr_boids[j]);
                curr_x[j] = birds.m_curr_boids[j].getX();
                curr_y[j] = birds.m_curr_boids[j].getY();
                curr_xvel[j] = birds.m_curr_boids[j].getXvel();
                curr_yvel[j] = birds.m_curr_boids[j].getYvel();
                curr_id[j] = birds.m_curr_boids[j].getid();
            }
         }
            
        else if (rank == MASTER){          
            for (int j = 0; j < boid_count; j ++) {
                    birds.advance(birds.m_curr_boids[j]); 
                    MASTERcurr_x[j] = birds.m_curr_boids[j].getX();
                    MASTERcurr_y[j] = birds.m_curr_boids[j].getY();
                    MASTERcurr_xvel[j] = birds.m_curr_boids[j].getXvel();
                    MASTERcurr_yvel[j] = birds.m_curr_boids[j].getYvel();
                    MASTERcurr_id[j] = birds.m_curr_boids[j].getid();
            }
            RECV_x = (double *)malloc((NUM_BOIDS-boid_count)*sizeof(double));
            RECV_y = (double *)malloc((NUM_BOIDS-boid_count)*sizeof(double));
            RECV_xvel = (double *)malloc((NUM_BOIDS-boid_count)*sizeof(double));
            RECV_yvel = (double *)malloc((NUM_BOIDS-boid_count)*sizeof(double));
            RECV_id = (int *)malloc((NUM_BOIDS-boid_count)*sizeof(int));
        }

        err = MPI_Gatherv(curr_x, boid_count, MPI_DOUBLE, RECV_x, worker_sizes, displs, MPI_DOUBLE, MASTER, MPI_COMM_WORLD);
        err = MPI_Gatherv(curr_y, boid_count, MPI_DOUBLE, RECV_y, worker_sizes, displs, MPI_DOUBLE, MASTER, MPI_COMM_WORLD);       
        err = MPI_Gatherv(curr_xvel, boid_count, MPI_DOUBLE, RECV_xvel, worker_sizes, displs, MPI_DOUBLE, MASTER, MPI_COMM_WORLD);        
        err = MPI_Gatherv(curr_yvel, boid_count, MPI_DOUBLE, RECV_yvel, worker_sizes, displs, MPI_DOUBLE, MASTER, MPI_COMM_WORLD); 
        err = MPI_Gatherv(curr_id, boid_count, MPI_INT, RECV_id, worker_sizes, displs, MPI_INT, MASTER, MPI_COMM_WORLD);        
   
        if (rank == MASTER){               
            for (int j = 0; j < NUM_BOIDS; j++){

                int id;                
                int k = NUM_BOIDS-boid_count;
                
                if (j < k){
                    id = RECV_id[j];
                    birds.m_boids[id].update(RECV_x[j], RECV_y[j] , RECV_xvel[j] , RECV_yvel[j] , RECV_id[j] ); 
                }
 
                else if(j >= NUM_BOIDS-boid_count) {
                    id = MASTERcurr_id[j-k];
                    birds.m_boids[id].update(MASTERcurr_x[j-k], MASTERcurr_y[j-k] , MASTERcurr_xvel[j-k] , MASTERcurr_yvel[j-k] , MASTERcurr_id[j-k] );  
                }
            }

        }
   


    
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
