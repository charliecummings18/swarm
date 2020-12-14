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
    data = pd.read_csv("boid_data.csv")
    
    infoFile = pd.read_csv("infoFile.csv", header=None)

    HEIGHT = float(infoFile.iloc[1][0])  
    HEIGHT = int(HEIGHT)
    
    WIDTH = float(infoFile.iloc[1][1])
    WIDTH = int(WIDTH)
    TIME_LIMIT = float(infoFile.iloc[1][3])
    TIME_STEP= float(infoFile.iloc[1][4])
    
    FRAMES = float(TIME_LIMIT/TIME_STEP)
    FRAMES = int(FRAMES)
    INTERVAL = int(TIME_STEP*1000)
    
    fig = plt.figure()
    ax = plt.axes(xlim=(-WIDTH,WIDTH ), ylim=(-HEIGHT, HEIGHT) )


    particles, = ax.plot([], [], 'bo', ms=2)


    def init():
        particles.set_data([], [])
        return particles,

    #t = np.linspace(0,1000,1)
    def animate(t):

        
         
        x_positions = data.iloc[t,::2].to_list()
        y_positions = data.iloc[t,1::2].to_list() 
        
        x_positions.pop(50)


        particles.set_data(x_positions, y_positions) 

        return particles,

    anim = FuncAnimation(fig, animate, init_func=init,  frames = FRAMES, interval=INTERVAL)


    
    anim.save('boid.mp4', writer='ffmpeg')
    
    
    
    return 0
    
