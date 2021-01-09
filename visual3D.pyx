# distutils: language=c++
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation
import time


def main():
    start_time = time.time()
    data = pd.read_csv("boid_data3D.csv")
    
    infoFile = pd.read_csv("infoFile3D.csv", header=None)

    cdef:
        int HEIGHT = int(float(infoFile.iloc[1][0]))  
        int WIDTH = int(float(infoFile.iloc[1][1]))
        int DEPTH = int(float(infoFile.iloc[1][2]))
        int TIME_LIMIT = int(float(infoFile.iloc[1][4]))
        float TIME_STEP= float(infoFile.iloc[1][5])
        int NUM_BOIDS = int(float(infoFile.iloc[1][7]))
        int FRAMES = int(float(TIME_LIMIT/TIME_STEP)) 
        int INTERVAL = int(TIME_STEP*1000)
        
    fig = plt.figure()
    ax = p3.Axes3D(fig)
    x_positions = data.iloc[0,::3].to_list()

    x_positions.pop(100)

    x_positions = np.array(x_positions)




    x=np.zeros(NUM_BOIDS)
    y=np.zeros(NUM_BOIDS)
    z=np.zeros(NUM_BOIDS)

    ax.set_xlim3d(-400, 400)
    ax.set_xlabel('X')

    ax.set_ylim3d(-250, 250)
    ax.set_ylabel('Y')

    ax.set_zlim3d(-250, 250)
    ax.set_zlabel('Z')
    
    points, = ax.plot(x, y, z, 'o', ms=2)
    txt = fig.suptitle('')  


    def update_points(t, x, y, z, points):
        txt.set_text('Time: {:d} ms'.format(t)) 

        x_positions = data.iloc[t,::3].to_list()
        x_positions.pop(100)
        
        y_positions = data.iloc[t,1::3].to_list()        
        z_positions = data.iloc[t,2::3].to_list()  
        
        new_x = np.array(x_positions)
        new_y = np.array(y_positions)
        new_z = np.array(z_positions)
        
        # update properties
        points.set_data(new_x,new_y)
        points.set_3d_properties(new_z, 'z')
        
        # return modified artists
        
        return points,txt
      
    
    ani=animation.FuncAnimation(fig, update_points, frames=FRAMES, interval=INTERVAL, repeat = False, cache_frame_data = False, fargs=(x, y, z, points))
    plt.show()    
    ani.save('boid3D_1.mp4', writer='ffmpeg')

    
    
    
    return 0
    
