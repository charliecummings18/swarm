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

    fig = plt.figure()
    ax = plt.axes(xlim=(-100, 100), ylim=(-100, 100))


    particles, = ax.plot([], [], 'bo', ms=6)


    def init():
        particles.set_data([], [])
        return particles,


    def animate(t):

        
         
        x_positions = data.iloc[t,::2].to_list()
        y_positions = data.iloc[t,1::2].to_list() 
        
        x_positions.pop(50)


        particles.set_data(x_positions, y_positions) 

        return particles,

    anim = FuncAnimation(fig, animate, init_func=init, interval=100)


    
    anim.save('boid.mp4', writer='ffmpeg')
    
    
    
    return 0
    
