#!/usr/bin/env python

from collections import namedtuple
import matplotlib.pyplot as plt
from matplotlib.patches import Arrow, Circle
from matplotlib.collections import PatchCollection
import numpy as np
import pdb


position = namedtuple("position", ['x', 'y'])

def length(x, y):
    return np.sqrt((x - 0.0)**2 + (y - 0.0)**2)

def dist(x_0, y_0, x_1, y_1):
    return np.sqrt((x_1 - x_0)**2 + (y_1 - y_0)**2)

def main():
    k_rep = 3.0
    goal = position(100,60)
    obstacles = [(40.0,30.0), (70.0, 40.0)]
    obstacle_radius = 5.0
    repulsion_radius = 5.0

    max_arrow = length(100.0, 100.0)
    
    # make array of x locations to hold vectors
    # x     = np.zeros(100*100, dtype=np.float).reshape((100,100))
    x_ind = np.zeros(100*100, dtype=np.float).reshape((100,100))
    # make array of y locations to hold vectors
    # y     = np.zeros(100*100, dtype=np.float).reshape((100,100))
    y_ind = np.zeros(100*100, dtype=np.float).reshape((100,100))

    # assign attraction values
    for i in range(100):
        for j in range(100):
            x_ind[i][j] = i
            y_ind[i][j] = j
    
    x = x_ind - goal.x
    y = y_ind - goal.y

    for obstacle in obstacles:
        distance  = np.sqrt(np.power(x_ind - obstacle[0], 2) + np.power(y_ind - obstacle[1], 2))
        x_direction = np.sign(obstacle[0] - x_ind)
        y_direction = np.sign(obstacle[1] - y_ind)

        in_obstacle    = distance <= obstacle_radius
        x[in_obstacle] = 0.0
        y[in_obstacle] = 0.0

        repulsed = np.logical_and(distance > obstacle_radius, distance <= (repulsion_radius + repulsion_radius))
        x_delta = k_rep * 0.5 * x_direction * ((x_ind - obstacle[0]) ** 2)
        y_delta = k_rep * 0.5 * y_direction * ((y_ind - obstacle[1]) ** 2)
        x[repulsed] = x[repulsed] + x_delta[repulsed]
        y[repulsed] = y[repulsed] + y_delta[repulsed]

    vector_lengths = np.sqrt(np.power(x,2) + np.power(y,2))/max_arrow
    dx = x/100.0 * -1.0
    dy = y/100.0 * -1.0

    fig, ax = plt.subplots()

    for start_y in [0, 5, 10, 15]:
        robot_x = [0.0]
        robot_y = [float(start_y)]
        c = ['r', 'b', 'g', 'y'][int(start_y/5)]
        while True:
            d = dist(goal[0], goal[1], robot_x[-1], robot_y[-1])
            print("distance={}".format(d))
            if d < 0.1 or robot_x[-1] >= 100 or robot_y[-1] >= 100 or len(robot_x) > 1000:
                break
            x_ind = int(robot_x[-1])
            y_ind = int(robot_y[-1])
            robot_x.append(robot_x[-1] + dx[x_ind][y_ind])
            robot_y.append(robot_y[-1] + dy[x_ind][y_ind])
        ax.scatter(robot_x, robot_y, s=2.0, c=c)

    cell_diagonal = np.sqrt(2.0) / 2.0
    offsets = (cell_diagonal-vector_lengths/2.0) * .5
    arrows = []
    for i in range(100):
        for j in range(100):
            arrows.append(Arrow(i + offsets[i][j], j + offsets[i][j], dx[i][j], dy[i][j], vector_lengths[i][j], width=.00001))
    ax.add_collection(PatchCollection(arrows))
    ax.set_aspect('equal', 'datalim')
    # ax.quiver(x_ind, y_ind, dx, dy, scale=100)
    plt.ylim((-10, 110))
    plt.xlim((-10, 110))


    plt.savefig('a3_field_potential.png', dpi=600)
    plt.show()
    return

if __name__=='__main__':
    main()
    
