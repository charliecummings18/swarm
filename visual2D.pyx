# distutils: language=c++
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.animation import FuncAnimation
import time
import numpy as np
from libcpp.vector cimport vector
from cython.parallel cimport prange
from libcpp.list cimport list
cimport openmp

def main():
    start_time = time.time()
    data = pd.read_csv("boid_data2D.csv")
    
    infoFile = pd.read_csv("infoFile2D.csv", header=None)

    cdef:
        int HEIGHT = int(float(infoFile.iloc[1][0]))  
        int WIDTH = int(float(infoFile.iloc[1][1]))
        int TIME_LIMIT = int(float(infoFile.iloc[1][3]))
        float TIME_STEP= float(infoFile.iloc[1][4])
        int NUM_BOIDS = int(float(infoFile.iloc[1][12]))
        int FRAMES = int(float(TIME_LIMIT/TIME_STEP)) 
        int INTERVAL = int(TIME_STEP*1000)
        
    
    fig = plt.figure()
    ax = plt.axes(xlim=(-WIDTH,WIDTH ), ylim=(-HEIGHT, HEIGHT) )


    particles, = ax.plot([], [], 'o', ms=1)


    def init():
        particles.set_data([], [])
        return particles,

    def animate(t):
        
        x_positions = data.iloc[t,::2].to_list()
        y_positions = data.iloc[t,1::2].to_list() 
        
        x_positions.pop(NUM_BOIDS)


        particles.set_data(x_positions, y_positions) 

        return particles,

    anim = FuncAnimation(fig, animate, init_func=init,  frames = FRAMES, interval=INTERVAL)


    
    #anim.save('boid2D.mp4', writer='ffmpeg')
    plt.show()
    
    
    return 0
    