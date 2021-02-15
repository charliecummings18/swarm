# distutils: language=c++
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.animation import FuncAnimation
import time
import numpy as np
from libcpp.vector cimport vector
from cython.parallel cimport prange


def main():
    start_time = time.time()
    data = pd.read_csv("boid_data2D.csv")
    
    infoFile = pd.read_csv("infoFile2D.csv", header=None)

    cdef:
        int HEIGHT = int(float(infoFile.iloc[1][0]))  
        int WIDTH = int(float(infoFile.iloc[1][1]))
        int TIME_LIMIT = int(float(infoFile.iloc[1][3]))
        float TIME_STEP= float(infoFile.iloc[1][4])
        int NUM_BOIDS = int(float(infoFile.iloc[1][5]))
        int FRAMES = int(float(TIME_LIMIT/TIME_STEP)) 
        int INTERVAL = int(TIME_STEP*1000)
        int PREDATORS = int(float(infoFile.iloc[1][12]))
        
    
    fig = plt.figure()
    ax = plt.axes(xlim=(-WIDTH,WIDTH ), ylim=(-HEIGHT, HEIGHT) )

    colours = []

    for i in range(0,NUM_BOIDS):
        if (i < PREDATORS):
            colours.append('red')
        else:
            colours.append('black')


            


    particles = ax.scatter([], [], color=[], s =1)

    def init():
        particles.set_offsets([])
        return particles,

    def animate(t):
        
        x_positions = data.iloc[t,::2].to_list()
        y_positions = data.iloc[t,1::2].to_list() 
        
        x_positions.pop(NUM_BOIDS)
        
        particles.set_offsets(np.c_[x_positions,y_positions]) 
        particles.set_color(colours)
        return particles,
    anim = FuncAnimation(fig, animate, init_func=init,  frames = FRAMES, interval=INTERVAL)


    plt.show()
    anim.save('boid2D.mp4', writer='ffmpeg')  
    
    return 0
    
