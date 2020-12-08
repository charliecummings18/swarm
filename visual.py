import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.animation import FuncAnimation
import numpy as np

plt.style.use('seaborn-pastel')

data = pd.read_csv("boid_data.csv")
######

x_positions = []
y_positions = []


######


fig = plt.figure()
ax = plt.axes(xlim=(-10, 10), ylim=(-10, 10))


particles, = ax.plot([], [], 'bo', ms=6)

def init():
    particles.set_data([], [])
    return particles,


def animate(t):
    
    x_positions.clear()
    y_positions.clear()
    for i in range(0,data.shape[1]-1):
        if (i % 2 ==0):
            x_positions.append(data.loc[t][i])
        else:
            y_positions.append(data.loc[t][i])
    print(x_positions)
    particles.set_data(x_positions, y_positions) 
     
    return particles,

anim = FuncAnimation(fig, animate, init_func=init,
                               frames=20, interval=50, blit=True)


anim.save('boid1.mp4', writer='imagemagick')


