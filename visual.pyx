import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.animation import FuncAnimation
import numpy as np
import time


def main():
    
#    cdef:
#        float start_time, elapsed_time, x_positions, y_positions
        
    
    start_time = time.time()

    #plt.style.use('seaborn-pastel')

    data = pd.read_csv("boid_data.csv")
    ######

    arr_size = (data.shape[1] - 1)/2
    arr_size = int(arr_size)
    x_positions = np.empty([arr_size])
    y_positions = np.empty([arr_size])


    ######


    fig = plt.figure()
    ax = plt.axes(xlim=(-100, 100), ylim=(-100, 100))


    particles, = ax.plot([], [], 'bo', ms=6)

    def init():
        particles.set_data([], [])
        return particles,


    def animate(t):
    

        i_x = 0
        i_y = 0

        for i in range(0,data.shape[1]-1):
            if (i % 2 ==0):
                x_positions[i_x] = data.loc[t][i]
                i_x+=1
            else:
                y_positions[i_y] = data.loc[t][i]
                i_y+=1
                particles.set_data(x_positions, y_positions) 
     
        return particles,

    anim = FuncAnimation(fig, animate, init_func=init, interval=100, blit=True)


    anim.save('boid1.mp4', writer='imagemagick')
    print("Animation Saved\n")
    elapsed_time = time.time() - start_time

    print("Time Elapsed: ", elapsed_time,'\n')
    
    return 0
    
