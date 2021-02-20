#include <iostream>
#include <stdlib.h> 
#include <stdio.h>
#include <vector>
#include <functional>
#include <tuple>
#include <cmath>
#include <fstream>
#include <string>
#include "parameters.h"
#include "boid.cpp"
#include "flock.cpp"
#include <chrono>

int main(int argc, char *argv[]) {


    int k;  
    auto initial = std::chrono::high_resolution_clock::now();
    
    
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
    
    // Generate birds in the flock with random positions 
    birds.generate(NUM_BOIDS,"SERIAL");

 
     
    // Write infoFile.csv file, this contains all relevant data which visual_2D/3D.pyx
    // needs to read

    infoFile << "HEIGHT, WIDTH, DEPTH, MAX_SPEED, TIME_LIMIT, TIME_STEP, NUM_BOIDS, ALIGN_VISIBILITY, \
                    COHESION_VISIBILITY, SEPERATION_VISIBILITY, ALIGN_FORCE,  COHESION_FORCE, SEPERATION_FORCE, PREDATORS\n";
    
    infoFile << std::to_string(HEIGHT) + "," + std::to_string(DEPTH) + "," + 
    std::to_string(WIDTH) + "," + std::to_string(MAX_SPEED) \
    + "," + std::to_string(TIME_LIMIT) + "," + std::to_string(TIME_STEP) + "," \
    + std::to_string(NUM_BOIDS) + "," + std::to_string(ALIGN_VISIBILITY) + "," \
    + std::to_string(COHESION_VISIBILITY) + "," + std::to_string(SEPERATION_VISIBILITY)\
    + "," + std::to_string(ALIGN_FORCE) +"," + std::to_string(COHESION_FORCE) + ","\
    + std::to_string(SEPERATION_FORCE) + ","\
    + std::to_string(PREDATORS); 


    
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


    
    // Advance the birds   
    double time = 0;
    while (time < TIME_LIMIT) {
        for (int num = 0; num < NUM_BOIDS; num ++) {
            if (DIMENSIONS == 2){
            data << birds.m_curr_boids[num].getX() <<"," 
                 << birds.m_curr_boids[num].getY() <<","; 
            }
            else if (DIMENSIONS == 3){
            data << birds.m_curr_boids[num].getX() <<"," 
                 << birds.m_curr_boids[num].getY() <<","
                 << birds.m_curr_boids[num].getZ() <<","; 
            }
        }
        data << '\n';

    

        for (k = 0; k < NUM_BOIDS; k++) {
            birds.advance(birds.m_curr_boids[k]);
        }
    time += TIME_STEP;
    }
    
    data.close();
    
    
    auto final = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(final - initial); 
    //printf("Total Elapsed-SERIAL: %8.6f s\n",final-initial); 
    std::cout<< "Total Elapsed-Serial: " << duration.count() << 's' <<'\n';
    
    
    return 0;
    
}
