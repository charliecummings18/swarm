#include <iostream>
#include <stdlib.h> 
#include <stdio.h>
#include <ctime>
#include <vector>
#include <functional>
#include <mpi.h>
#include <omp.h>
#include <tuple>
#include <cmath>
#include <fstream>
#include <string>
#include "parameters.h"
#include "boid.cpp"
#include "flock.cpp"
    

int main(int argc, char *argv[]) {

    if (argc < 2) {
        printf("Usage: \n -np [number of processes] ./swarmMIXED [number of threads]\n");
        return 1;
    }
    int k, size, rank, err;
    double wtime, chunksize;   
    
    int threads = atoi(argv[1]);
    omp_set_num_threads( threads );
    
    err = MPI_Init ( &argc, &argv );
    err = MPI_Comm_size ( MPI_COMM_WORLD, &size );
    err = MPI_Comm_rank ( MPI_COMM_WORLD, &rank );
    wtime = MPI_Wtime();
    
    chunksize = (2*WIDTH)/(double(size));
    
    // Initialise data file
    // Initialise file with generic info e.g. constants, forces

    std::string fileName;
    std::string infoFileName;   
    infoFileName = "infoFile.csv"; 
    if (DIMENSIONS == 2){
        fileName = "boid_data2D.csv";        
    }
    else if (DIMENSIONS == 3){
        fileName = "boid_data3D.csv";  
    }
    std::ofstream data(fileName);    
    std::ofstream infoFile(infoFileName);    
    

    // Create flock (in this case birds)
    Flock birds{};
    birds.flockSize(NUM_BOIDS);
    
    // MASTER generates birds in the flock with random positions
    if (rank == MASTER){  
        birds.generate(NUM_BOIDS,"HYBRID");

    // Write infoFile.csv file, this contains all relevant data which visual_2D/3D.pyx
    // needs to read

        

    infoFile << "HEIGHT, WIDTH, DEPTH, MAX_SPEED, TIME_LIMIT, TIME_STEP, NUM_BOIDS, ALIGN_VISIBILITY, \
                    COHESION_VISIBILITY, SEPERATION_VISIBILITY, ALIGN_FORCE,  COHESION_FORCE, SEPERATION_FORCE, PREDATORS\n";
    
    infoFile << std::to_string(HEIGHT) + "," + std::to_string(DEPTH) + "," + std::to_string(WIDTH) + "," + std::to_string(MAX_SPEED) \
    + "," + std::to_string(TIME_LIMIT) + "," + std::to_string(TIME_STEP) + "," \
    + std::to_string(NUM_BOIDS) + "," + std::to_string(ALIGN_VISIBILITY) + "," \
    + std::to_string(COHESION_VISIBILITY) + "," + std::to_string(SEPERATION_VISIBILITY)\
    + "," + std::to_string(ALIGN_FORCE) +"," + std::to_string(COHESION_FORCE) + ","\
    + std::to_string(SEPERATION_FORCE) + ","\
    +std::to_string(PREDATORS); 

                

    // Write header columns for boid_data2D.csv file

            for (int num = 0; num < NUM_BOIDS; num ++) {   
                if (DIMENSIONS == 2){
                    data << "boid" + std::to_string(num) + ".x, boid" 
                                   + std::to_string(num) + ".y, ";   
                }
                else if (DIMENSIONS == 3){
                    data << "boid" + std::to_string(num) + ".x, boid"
                                   + std::to_string(num) + ".y, boid"
                                   + std::to_string(num) + ".z, ";   
                }                
            }
            data << '\n';

        
    }
      
    double time = 0;            

    while (time < TIME_LIMIT) {
        
        int boid_count;
        
        double *curr_x = (double *)malloc(NUM_BOIDS*sizeof(double));
        double *curr_y = (double *)malloc(NUM_BOIDS*sizeof(double));
        double *curr_xvel = (double *)malloc(NUM_BOIDS*sizeof(double));
        double *curr_yvel = (double *)malloc(NUM_BOIDS*sizeof(double));
        int *curr_id = (int *)malloc(NUM_BOIDS*sizeof(int));
        double *curr_z = (double *)malloc(NUM_BOIDS*sizeof(double));        
        double *curr_zvel = (double *)malloc(NUM_BOIDS*sizeof(double));        
        
        double *MASTERcurr_x = (double *)malloc(NUM_BOIDS*sizeof(double));
        double *MASTERcurr_y = (double *)malloc(NUM_BOIDS*sizeof(double));
        double *MASTERcurr_xvel = (double *)malloc(NUM_BOIDS*sizeof(double));
        double *MASTERcurr_yvel = (double *)malloc(NUM_BOIDS*sizeof(double));
        int *MASTERcurr_id = (int *)malloc(NUM_BOIDS*sizeof(int));
        double *MASTERcurr_z = (double *)malloc(NUM_BOIDS*sizeof(double));        
        double *MASTERcurr_zvel = (double *)malloc(NUM_BOIDS*sizeof(double));        
        
        double *RECV_x;
        double *RECV_y;
        double *RECV_xvel;
        double *RECV_yvel;
        int    *RECV_id;
        double *RECV_z;
        double *RECV_zvel;
        
        
        int *worker_sizes = (int *)malloc((size)*sizeof(int));
        int *displs = (int *)malloc((size)*sizeof(int));

        MPI_Request Request;            
        MPI_Status Status;
               
/////////////////// MASTER PROCESS ///////////////////
//  MASTER distributes slice of program (based on x position) to each worker
//  MASTER takes the first slice of boids and adds these to the curr_boids list 
        if (rank == MASTER) {
            for (int num = 0; num < NUM_BOIDS; num ++) {
                if (DIMENSIONS == 2){
                data << birds.m_boids[num].getX() <<"," 
                     << birds.m_boids[num].getY() <<","; 
                }
                else if (DIMENSIONS == 3){
                data << birds.m_boids[num].getX() <<"," 
                     << birds.m_boids[num].getY() <<","
                     << birds.m_boids[num].getZ() <<","; 
                }
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
                        if (DIMENSIONS == 3){
                            MASTERcurr_z[index] = birds.m_boids[k].getZ();                        
                            MASTERcurr_zvel[index] = birds.m_boids[k].getZvel();
                        }
                        index +=1;
                        
                    }
                
                    else if (assigned_rank == (size-1) && birds.m_boids[k].getX() > (-WIDTH + chunksize*assigned_rank) ) {
                        curr_x[index] = birds.m_boids[k].getX();
                        curr_y[index] = birds.m_boids[k].getY();
                        curr_xvel[index] = birds.m_boids[k].getXvel();
                        curr_yvel[index] = birds.m_boids[k].getYvel();
                        curr_id[index] = birds.m_boids[k].getid();
                        if (DIMENSIONS == 3){                       
                            curr_z[index] = birds.m_boids[k].getZ();                        
                            curr_zvel[index] = birds.m_boids[k].getZvel();   
                        }                        
                        index +=1;
                    }
                
                    else if (birds.m_boids[k].getX() < (-WIDTH + chunksize*(assigned_rank+1)) && birds.m_boids[k].getX() > (-WIDTH + chunksize*assigned_rank) ) {
                        curr_x[index] = birds.m_boids[k].getX();
                        curr_y[index] = birds.m_boids[k].getY();
                        curr_xvel[index] = birds.m_boids[k].getXvel();
                        curr_yvel[index] = birds.m_boids[k].getYvel();
                        curr_id[index] = birds.m_boids[k].getid();
                        if (DIMENSIONS == 3){                       
                            curr_z[index] = birds.m_boids[k].getZ();                        
                            curr_zvel[index] = birds.m_boids[k].getZvel();  
                        }                  
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
                    
                    if (DIMENSIONS == 3){
                        err = MPI_Send(curr_z, index, MPI_DOUBLE, assigned_rank, assigned_rank, MPI_COMM_WORLD);                    
                        err = MPI_Send(curr_zvel, index, MPI_DOUBLE, assigned_rank, assigned_rank, MPI_COMM_WORLD);                        
                    }
                    
 
                }
   
                else if (assigned_rank == MASTER){
                    boid_count = index;
                }
  
                assigned_rank+=1;
                x_chunk+=chunksize;
                
            } 
        }
        
//      WORKERS receive boids sent from MASTER and add to their own curr_boids list
        else if (rank != MASTER) {
            
            err = MPI_Recv(&boid_count, 1, MPI_INT, MASTER, rank, MPI_COMM_WORLD, &Status);

            err = MPI_Recv(curr_x, boid_count, MPI_DOUBLE, MASTER, rank, MPI_COMM_WORLD, &Status);
            err = MPI_Recv(curr_y, boid_count, MPI_DOUBLE, MASTER, rank, MPI_COMM_WORLD, &Status);           
            err = MPI_Recv(curr_xvel, boid_count, MPI_DOUBLE, MASTER, rank, MPI_COMM_WORLD, &Status);            
            err = MPI_Recv(curr_yvel, boid_count, MPI_DOUBLE, MASTER, rank, MPI_COMM_WORLD, &Status);            
            err = MPI_Recv(curr_id, boid_count, MPI_INT, MASTER, rank, MPI_COMM_WORLD, &Status); 
            
            if (DIMENSIONS == 3){
                err = MPI_Recv(curr_z, boid_count, MPI_DOUBLE, MASTER, rank, MPI_COMM_WORLD, &Status);           
                err = MPI_Recv(curr_zvel, boid_count, MPI_DOUBLE, MASTER, rank,  MPI_COMM_WORLD, &Status);               
            }
        }

        birds.m_curr_boids.clear();
        
        
        if (rank != MASTER){    
            for (int i = 0; i < boid_count; i++){
                if (DIMENSIONS == 2){
                    birds.curr_boids(curr_x[i], curr_y[i], curr_xvel[i], curr_yvel[i], curr_id[i]);
                }
                else if (DIMENSIONS == 3){
                    birds.curr_boids(curr_x[i], curr_y[i], curr_xvel[i], curr_yvel[i], curr_id[i], curr_z[i], curr_zvel[i]);
                }                
            }
        }
        
        else if (rank == MASTER){    
            for (int i = 0; i < boid_count; i++){
                if (DIMENSIONS == 2){
                    birds.curr_boids(MASTERcurr_x[i], MASTERcurr_y[i], MASTERcurr_xvel[i], MASTERcurr_yvel[i], MASTERcurr_id[i]);
                }
                else if (DIMENSIONS == 3){
                    birds.curr_boids(MASTERcurr_x[i], MASTERcurr_y[i], MASTERcurr_xvel[i], MASTERcurr_yvel[i], MASTERcurr_id[i], MASTERcurr_z[i], MASTERcurr_zvel[i]);
                }                
            }
        }
   
      
///////////////////  COMMUNICATION ///////////////////
// If a boid is near the boundary of the neighbouring process, its information is sent to 
// the neighbour. This boid is then added to the neighbouring processes list of neighbours // so that the boid feels the correct force from its neighbours.
        
        int left_count = 0;
        int right_count = 0;
        
        if (rank != size-1){
            
            // SENDING TO RIGHT NEIGHBOUR PROCESS//
            std::vector<int>temp_send_right_id;
            std::vector<std::vector<double>>temp_send_right;
            
            for (int i = 0; i<DIMENSIONS*2 ; i++){
                temp_send_right.push_back(std::vector<double>());
            }
            int index = 0;

            double *send_right[DIMENSIONS*2];

            int *send_right_id;
            int right_rank = rank+1;
               
            for (int r = 0; r < boid_count; r++) {
             
                if ( ((-WIDTH + (chunksize*(rank+1))) - birds.m_curr_boids[r].getX())   < VISIBILITY ){
                    
                    temp_send_right[0].push_back(birds.m_curr_boids[r].getX());
                    temp_send_right[1].push_back(birds.m_curr_boids[r].getY());
                    temp_send_right[2].push_back(birds.m_curr_boids[r].getXvel());
                    temp_send_right[3].push_back(birds.m_curr_boids[r].getYvel());
                    temp_send_right_id.push_back(birds.m_curr_boids[r].getid());
                    
                    if (DIMENSIONS == 3){
                        temp_send_right[4].push_back(birds.m_curr_boids[r].getZ());
                        temp_send_right[5].push_back(birds.m_curr_boids[r].getZvel()); 
                    }
                
 
                }
                
            
            }
             
            int send_right_size = temp_send_right_id.size();
            send_right_id = (int *)malloc(send_right_size*sizeof(int));
            for (int i = 0; i < DIMENSIONS*2; i++){        
                send_right[i] = (double *)malloc(send_right_size * sizeof(double)); 
            }
             
            for (int i = 0; i < send_right_size; i++) {
                send_right[0][i] = temp_send_right[0][i];
                send_right[1][i] = temp_send_right[1][i];
                send_right[2][i] = temp_send_right[2][i];
                send_right[3][i] = temp_send_right[3][i];
                send_right_id[i] = temp_send_right_id[i];
                if (DIMENSIONS == 3){
                    send_right[4][i] = temp_send_right[4][i];
                    send_right[5][i] = temp_send_right[5][i];                   
                    
                }
            }
        
            err = MPI_Send(&send_right_size, 1, MPI_INT, right_rank , 99, MPI_COMM_WORLD);
            
            err = MPI_Send(send_right[0], send_right_size, MPI_DOUBLE, right_rank, 98, MPI_COMM_WORLD);
            err = MPI_Send(send_right[1], send_right_size, MPI_DOUBLE, right_rank, 97, MPI_COMM_WORLD);           
            err = MPI_Send(send_right[2], send_right_size, MPI_DOUBLE, right_rank, 96, MPI_COMM_WORLD);            
            err = MPI_Send(send_right[3], send_right_size, MPI_DOUBLE, right_rank, 95, MPI_COMM_WORLD);            
            
            err = MPI_Send(send_right_id, send_right_size, MPI_INT, right_rank, 94, MPI_COMM_WORLD);
            if (DIMENSIONS == 3){
                err = MPI_Send(send_right[4], send_right_size, MPI_DOUBLE, right_rank, 70, MPI_COMM_WORLD);            
                err = MPI_Send(send_right[5], send_right_size, MPI_DOUBLE, right_rank, 71, MPI_COMM_WORLD);        
            }
            
        
            
            
            // RECIEVING FROM RIGHT NEIGHBOUR PROCESS//
            

     
            err = MPI_Recv(&right_count, 1, MPI_INT, right_rank, 93, MPI_COMM_WORLD, &Status);


            double *rec_right[DIMENSIONS*2];
   
            int *rec_right_id = (int *)malloc(right_count*sizeof(int));
            for (int i = 0; i < DIMENSIONS*2; i++){        
                rec_right[i] = (double *)malloc(right_count* sizeof(double)); 
            }
            
            err = MPI_Recv(rec_right[0], right_count, MPI_DOUBLE, right_rank, 92, MPI_COMM_WORLD, &Status);
            err = MPI_Recv(rec_right[1], right_count, MPI_DOUBLE, right_rank, 91, MPI_COMM_WORLD, &Status);
            err = MPI_Recv(rec_right[2], right_count, MPI_DOUBLE, right_rank, 90, MPI_COMM_WORLD, &Status);
            err = MPI_Recv(rec_right[3], right_count, MPI_DOUBLE, right_rank, 89, MPI_COMM_WORLD, &Status);
            err = MPI_Recv(rec_right_id, right_count, MPI_INT, right_rank, 88, MPI_COMM_WORLD, &Status);
            if (DIMENSIONS == 3){
                err = MPI_Recv(rec_right[4], right_count, MPI_DOUBLE, right_rank, 72, MPI_COMM_WORLD, &Status);
                err = MPI_Recv(rec_right[5], right_count, MPI_DOUBLE, right_rank, 73, MPI_COMM_WORLD, &Status); 
            }
            
            
            for (int r = 0; r < right_count; r++) {
                if (DIMENSIONS == 2){
                    birds.curr_boids(rec_right[0][r],rec_right[1][r],rec_right[2][r],rec_right[3][r],rec_right_id[r]);     
                }
                else if (DIMENSIONS == 3){
                     birds.curr_boids(rec_right[0][r],rec_right[1][r],rec_right[2][r],rec_right[3][r],rec_right_id[r], rec_right[4][r], rec_right[5][r]);                     
                }
            } 
            
        }
            
            
        if (rank != MASTER){
            
            
            // SENDING TO LEFT NEIGHBOUR PROCESS//
            
            std::vector<int>temp_send_left_id;
            std::vector<std::vector<double>>temp_send_left;
            
            for (int i = 0; i<DIMENSIONS*2 ; i++){
                temp_send_left.push_back(std::vector<double>());
            }

            double *send_left[DIMENSIONS*2];

            
            int *send_left_id;
            int left_rank = rank-1;
            for (int r = 0; r < boid_count; r++) {
            
                if ( (birds.m_curr_boids[r].getX() - (-WIDTH + (chunksize*rank)))  < VISIBILITY ){
                    
                    temp_send_left[0].push_back(birds.m_curr_boids[r].getX());
                    temp_send_left[1].push_back(birds.m_curr_boids[r].getY());
                    temp_send_left[2].push_back(birds.m_curr_boids[r].getXvel());
                    temp_send_left[3].push_back(birds.m_curr_boids[r].getYvel());
                    temp_send_left_id.push_back(birds.m_curr_boids[r].getid());
                    
                    if (DIMENSIONS == 3){
                        temp_send_left[4].push_back(birds.m_curr_boids[r].getZ());
                        temp_send_left[5].push_back(birds.m_curr_boids[r].getZvel());                       
                    }
                } 
            }
            int send_left_size = temp_send_left_id.size();
            send_left_id = (int *)malloc(send_left_size*sizeof(int));
            for (int i = 0; i < DIMENSIONS*2; i++){        
                send_left[i] = (double *)malloc(send_left_size* sizeof(double)); 
            }
            
            for (int i = 0; i < send_left_size; i++) {
                send_left[0][i] = temp_send_left[0][i];
                send_left[1][i] = temp_send_left[1][i];
                send_left[2][i] = temp_send_left[2][i];
                send_left[3][i] = temp_send_left[3][i];
                send_left_id[i] = temp_send_left_id[i];
                if (DIMENSIONS == 3){
                    send_left[4][i] = temp_send_left[4][i];
                    send_left[5][i] = temp_send_left[5][i];                    
                }
            }
        
            err = MPI_Send(&send_left_size, 1, MPI_INT,left_rank , 93, MPI_COMM_WORLD);
            
            err = MPI_Send(send_left[0], send_left_size, MPI_DOUBLE, left_rank, 92, MPI_COMM_WORLD);
            err = MPI_Send(send_left[1], send_left_size, MPI_DOUBLE, left_rank, 91, MPI_COMM_WORLD);
            err = MPI_Send(send_left[2], send_left_size, MPI_DOUBLE, left_rank, 90, MPI_COMM_WORLD);
            err = MPI_Send(send_left[3], send_left_size, MPI_DOUBLE, left_rank, 89, MPI_COMM_WORLD);
            err = MPI_Send(send_left_id, send_left_size, MPI_INT, left_rank, 88, MPI_COMM_WORLD);           
            
            if (DIMENSIONS == 3){
                err = MPI_Send(send_left[4], send_left_size, MPI_DOUBLE, left_rank, 72, MPI_COMM_WORLD);
                err = MPI_Send(send_left[5], send_left_size, MPI_DOUBLE, left_rank, 73, MPI_COMM_WORLD);               
            }
            
            

            
            // RECIEVING FROM LEFT NEIGHBOUR PROCESS//

            
            
            err = MPI_Recv(&left_count, 1, MPI_INT, left_rank, 99, MPI_COMM_WORLD, &Status);
            double *rec_left[DIMENSIONS*2];
            int *rec_left_id = (int *)malloc(left_count*sizeof(int));
            for (int i = 0; i < DIMENSIONS*2; i++){        
                rec_left[i] = (double *)malloc(left_count* sizeof(double)); 
            }
            
            err = MPI_Recv(rec_left[0], left_count, MPI_DOUBLE, left_rank, 98, MPI_COMM_WORLD, &Status);
            err = MPI_Recv(rec_left[1], left_count, MPI_DOUBLE, left_rank, 97, MPI_COMM_WORLD, &Status);
            err = MPI_Recv(rec_left[2], left_count, MPI_DOUBLE, left_rank, 96, MPI_COMM_WORLD, &Status);
            err = MPI_Recv(rec_left[3], left_count, MPI_DOUBLE, left_rank, 95, MPI_COMM_WORLD, &Status);
            err = MPI_Recv(rec_left_id, left_count, MPI_INT, left_rank, 94, MPI_COMM_WORLD, &Status);
            
            if (DIMENSIONS == 3){
                err = MPI_Recv(rec_left[4], left_count, MPI_DOUBLE, left_rank, 70, MPI_COMM_WORLD, &Status);
                err = MPI_Recv(rec_left[5], left_count, MPI_DOUBLE, left_rank, 71, MPI_COMM_WORLD, &Status);               
            }
            
            
            for (int r = 0; r < left_count; r++) {
                if (DIMENSIONS == 2){
                    birds.curr_boids(rec_left[0][r],rec_left[1][r],rec_left[2][r],rec_left[3][r],rec_left_id[r]); 
                }
                else if (DIMENSIONS == 3){
                    birds.curr_boids(rec_left[0][r],rec_left[1][r],rec_left[2][r],rec_left[3][r],rec_left_id[r], rec_left[4][r], rec_left[5][r]); 
                }                
            }

        }
        
 /////////////////// END OF COMMUNICATION ///////////////////       

   
    
/////////////////// ADVANCE THE BOIDS ///////////////////
// MASTER and WORKERS advance each boid they control by one timestep, according to the 
// rules given in the FLOCK class.
// The curr arrays (which hold current boid data) are then updated, ready to be
// transferred back to MASTER

        if (rank != MASTER){   
            int j;
            #pragma omp parallel private(j), shared (curr_x,curr_y,curr_xvel, curr_yvel,curr_id)
            {
            #pragma omp for schedule(static) nowait

            for (int j = 0; j < boid_count; j ++) {
                birds.advance(birds.m_curr_boids[j]);
                curr_x[j] = birds.m_curr_boids[j].getX();
                curr_y[j] = birds.m_curr_boids[j].getY();
                curr_xvel[j] = birds.m_curr_boids[j].getXvel();
                curr_yvel[j] = birds.m_curr_boids[j].getYvel();
                curr_id[j] = birds.m_curr_boids[j].getid();
                
                if (DIMENSIONS == 3){
                    curr_z[j] = birds.m_curr_boids[j].getZ();
                    curr_zvel[j] = birds.m_curr_boids[j].getZvel();                   
                    }
                }
            }
    
         }
            
        else if (rank == MASTER){   
            int j;
            #pragma omp parallel private(j) shared (MASTERcurr_x,MASTERcurr_y,MASTERcurr_xvel, MASTERcurr_yvel,MASTERcurr_id)
            {
            #pragma omp for schedule(static) nowait
                
            for (j = 0; j < boid_count; j ++) {
                    birds.advance(birds.m_curr_boids[j]); 
                    MASTERcurr_x[j] = birds.m_curr_boids[j].getX();
                    MASTERcurr_y[j] = birds.m_curr_boids[j].getY();
                    MASTERcurr_xvel[j] = birds.m_curr_boids[j].getXvel();
                    MASTERcurr_yvel[j] = birds.m_curr_boids[j].getYvel();
                    MASTERcurr_id[j] = birds.m_curr_boids[j].getid();
                    if (DIMENSIONS == 3){
                        MASTERcurr_z[j] = birds.m_curr_boids[j].getZ();
                        MASTERcurr_zvel[j] = birds.m_curr_boids[j].getZvel();                       
              
                    }
                }
            }
            
            
            RECV_x = (double *)malloc((NUM_BOIDS-boid_count)*sizeof(double));
            RECV_y = (double *)malloc((NUM_BOIDS-boid_count)*sizeof(double));
            RECV_xvel = (double *)malloc((NUM_BOIDS-boid_count)*sizeof(double));
            RECV_yvel = (double *)malloc((NUM_BOIDS-boid_count)*sizeof(double));
            RECV_id = (int *)malloc((NUM_BOIDS-boid_count)*sizeof(int));
            if (DIMENSIONS == 3){
                RECV_z = (double *)malloc((NUM_BOIDS-boid_count)*sizeof(double));
                RECV_zvel = (double *)malloc((NUM_BOIDS-boid_count)*sizeof(double));                
            }
        }
        
/////////////////// GATHER BOIDS ///////////////////
// MPI_Gather is called to gather all boids from each process back to MASTER, which then 
// updates the main list of boids (m_boids)

        err = MPI_Gatherv(curr_x, boid_count, MPI_DOUBLE, RECV_x, worker_sizes, displs, MPI_DOUBLE, MASTER, MPI_COMM_WORLD);
        err = MPI_Gatherv(curr_y, boid_count, MPI_DOUBLE, RECV_y, worker_sizes, displs, MPI_DOUBLE, MASTER, MPI_COMM_WORLD);       
        err = MPI_Gatherv(curr_xvel, boid_count, MPI_DOUBLE, RECV_xvel, worker_sizes, displs, MPI_DOUBLE, MASTER, MPI_COMM_WORLD);        
        err = MPI_Gatherv(curr_yvel, boid_count, MPI_DOUBLE, RECV_yvel, worker_sizes, displs, MPI_DOUBLE, MASTER, MPI_COMM_WORLD); 
        err = MPI_Gatherv(curr_id, boid_count, MPI_INT, RECV_id, worker_sizes, displs, MPI_INT, MASTER, MPI_COMM_WORLD);   
        
        if (DIMENSIONS == 3){
            err = MPI_Gatherv(curr_z, boid_count, MPI_DOUBLE, RECV_z, worker_sizes, displs, MPI_DOUBLE, MASTER, MPI_COMM_WORLD);       
            err = MPI_Gatherv(curr_zvel, boid_count, MPI_DOUBLE, RECV_zvel, worker_sizes, displs, MPI_DOUBLE, MASTER, MPI_COMM_WORLD);              
        }
   
        if (rank == MASTER){               
            for (int j = 0; j < NUM_BOIDS; j++){

                int id;                
                int k = NUM_BOIDS-boid_count;
                
                if (j < k){
                    id = RECV_id[j];
                    if (DIMENSIONS == 2){
                        birds.m_boids[id].update(RECV_x[j], RECV_y[j] , RECV_xvel[j] , RECV_yvel[j] , RECV_id[j] ); 
                    }
                    else if (DIMENSIONS == 3){
                        birds.m_boids[id].update(RECV_x[j], RECV_y[j] , RECV_xvel[j] , RECV_yvel[j] , RECV_id[j] , RECV_z[j] , RECV_zvel[j]); 
                    }                    
                }
 
                else if(j >= NUM_BOIDS-boid_count) {
                    id = MASTERcurr_id[j-k];
                    if (DIMENSIONS == 2){
                        birds.m_boids[id].update(MASTERcurr_x[j-k], MASTERcurr_y[j-k] , MASTERcurr_xvel[j-k] , MASTERcurr_yvel[j-k] , MASTERcurr_id[j-k] );  
                    }
                    else if (DIMENSIONS == 3){
                        birds.m_boids[id].update(MASTERcurr_x[j-k], MASTERcurr_y[j-k] , MASTERcurr_xvel[j-k] , MASTERcurr_yvel[j-k] , MASTERcurr_id[j-k], MASTERcurr_z[j-k] , MASTERcurr_zvel[j-k] );  
                    }                    
                }
            }

        }

    time += TIME_STEP;
    }
    // Close file
    data.close();
    
    double total_time = MPI_Wtime() - wtime;       
    if (rank == MASTER) {
        printf("Program Complete.\n"); 
        printf("Total Elapsed-HYBRID: %8.6f s\n",total_time); 
    }
    
    MPI_Finalize();
    
    return 0;
    
}
