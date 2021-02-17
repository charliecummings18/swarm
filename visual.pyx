# distutils: language=c++
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.animation import FuncAnimation
import time
import numpy as np
from libcpp.vector cimport vector
from cython.parallel cimport prange
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation

def main():
    
    with open("parameters.h") as f:
        content = f.readlines()

    content = [x.strip() for x in content] 
    
    DIMENSION = content[24]
    
    infoFile = pd.read_csv("infoFile.csv", header=None)
    
    cdef:
            int HEIGHT = int(float(infoFile.iloc[1][0]))  
            int WIDTH = int(float(infoFile.iloc[1][1]))
            int DEPTH = int(float(infoFile.iloc[1][2]))
            int TIME_LIMIT = int(float(infoFile.iloc[1][4]))
            float TIME_STEP= float(infoFile.iloc[1][5])
            int NUM_BOIDS = int(float(infoFile.iloc[1][6]))
            int FRAMES = int(float(TIME_LIMIT/TIME_STEP)) 
            int INTERVAL = int(TIME_STEP*1000)
            int PREDATORS = int(float(infoFile.iloc[1][13]))
    
    
    if (DIMENSION == "const int DIMENSIONS = 2;"):
        start_time = time.time()
        data = pd.read_csv("boid_data2D.csv")
    
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
        
        
    elif (DIMENSION == "const int DIMENSIONS = 3;"):
        data = pd.read_csv("boid_data3D.csv")
        fig = plt.figure()
        ax = p3.Axes3D(fig)
        x_positions = data.iloc[0,::3].to_list()

        x_positions.pop(NUM_BOIDS)

        x_positions = np.array(x_positions)


        colours = []
        for i in range(0,NUM_BOIDS):
            if (i < PREDATORS):
                colours.append('red')
            else:
                colours.append('black')
            
            
            
        x=np.zeros(NUM_BOIDS)
        y=np.zeros(NUM_BOIDS)
        z=np.zeros(NUM_BOIDS)

        ax.set_xlim3d(-WIDTH, WIDTH)
        ax.set_xlabel('X')

        ax.set_ylim3d(-HEIGHT, HEIGHT)
        ax.set_ylabel('Y')

        ax.set_zlim3d(-DEPTH, DEPTH)
        ax.set_zlabel('Z')
        points = ax.scatter([], [], [], s=1, c='black')
        txt = fig.suptitle('')  


        def update_points(t, x, y, z, points):   
                
            txt.set_text('Time: {:d} ms'.format(t)) 

            x_positions = data.iloc[t,::3].to_list()
            x_positions.pop(NUM_BOIDS)
        
            y_positions = data.iloc[t,1::3].to_list()        
            z_positions = data.iloc[t,2::3].to_list()  
        
            new_x = np.array(x_positions)
            new_y = np.array(y_positions)
            new_z = np.array(z_positions)

        
            #points.set_offsets(np.c_[new_x,new_y])         
            #points.set_3d_properties(new_z, 'z')
            points._offsets3d = (new_x, new_y, new_z)
            #points.set_array(colours)
        
            return points,txt
      
    
        ani=animation.FuncAnimation(fig, update_points, frames=FRAMES,interval=INTERVAL, repeat = False, cache_frame_data = False, fargs=(x, y, z, points))
        plt.show()    
        ani.save('boid3D.mp4', writer='ffmpeg')
    
    return 0
    
