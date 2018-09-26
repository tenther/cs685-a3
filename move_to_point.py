#!/usr/bin/env python

from collections import namedtuple
import matplotlib.pyplot as plt
from matplotlib.patches import Arrow, Circle
from matplotlib.collections import PatchCollection
import numpy as np
import pdb

# Computer Science 685 Assignment 1 part 2
# Paul McKerley (G00616949)

Pose = namedtuple('pose', ['x', 'y', 'theta'])

def deg2rad(deg):
    return deg/180.0*np.pi

def rad2deg(rad):
    return rad*180.0/np.pi

def dist(x_0, y_0, x_1, y_1):
    return np.sqrt((x_1 - x_0)**2 + (y_1 - y_0)**2)

class DifferentialDriveRobot(object):
    def __init__(self, pose=None, l=1.0, k_v=0.5, k_h=4.0):
        self.l    = l
        self.pose = pose
        self.k_v  = k_v
        self.k_h  = k_h

    def moveToPoint(self, x_g, y_g):
        time_delta = 0.1
        x_plot = []
        y_plot = []
        theta_plot = []
        
        while True:
            pose = self.pose
        
            x_plot.append(pose.x)
            y_plot.append(pose.y)
            theta_plot.append(pose.theta)
            distance = dist(pose.x, pose.y, x_g, y_g)
            if distance < 1.0:
                break

            theta_g = np.arctan2(y_g - pose.y, x_g - pose.x)
            v = self.k_v * dist(pose.x,pose.y,x_g,y_g)
            omega = self.k_h * (theta_g - pose.theta)
            print("x={} y={} distance={} theta={} v={} omega={}".format(pose.x,pose.y,distance,pose.theta,v,omega))

            self.pose = Pose(pose.x + time_delta * v * np.cos(pose.theta), pose.y + time_delta * v * np.sin(pose.theta), pose.theta + time_delta * omega )

        return x_plot, y_plot, theta_plot

max_x = -np.inf
max_y = -np.inf
min_x = np.inf
min_y = np.inf
fig, ax = plt.subplots()

def plot(x,y, start, goal):
    global max_x
    global max_y
    global min_x
    global min_y

    x.append(start.x)
    x.append(goal.x)
    y.append(start.y)
    y.append(goal.y)

    max_x = max(max(x), max_x)
    max_y = max(max(y), max_y)
    min_x = min(min(x), min_x)
    min_y = min(min(y), min_y)

    x_range = max_x - min_x
    y_range = max_y - min_y

    if x_range < y_range:
        rng = y_range
    else:
        rng = x_range

    plt.scatter(x,y)
    start_circle = Circle((start.x, start.y), 1)
    start_circle.set_color((0,1,0))
    ax.set_xlim(min_x - 20, min_x + rng + 20)
    ax.set_ylim(min_y - 20, min_y + rng + 20)
    ax.set_aspect('equal', 'datalim')

    goal_circle = Circle((goal.x, goal.y), 1)
    goal_circle.set_color((1,0,0))

    start_arrow = Arrow(start.x, start.y, 10*np.cos(start.theta), 10*np.sin(start.theta), 2.0)
    goal_arrow = Arrow(goal.x, goal.y, 10*np.cos(goal.theta), 10*np.sin(goal.theta), 2.0)

    ax.add_collection(PatchCollection([start_circle, goal_circle, start_arrow, goal_arrow]))

def main():
    x_g = 0.0
    y_g = 0.0


    for initial_pose in [Pose(0,    50, deg2rad(180)),
                         Pose(50 * np.cos(np.pi/4), 50 * np.sin(np.pi/4), deg2rad(135)),
                         Pose(50,    0,  deg2rad(270)),
                         Pose(-50 * np.cos(np.pi/4), -50 * np.sin(np.pi/4), deg2rad(135)),
                         Pose(-50,   0,  deg2rad(180)),
                         ]:
        robot = DifferentialDriveRobot(pose=initial_pose)
        x,y, theta = robot.moveToPoint(x_g, y_g)
        plot(x, y, initial_pose, Pose(x_g, y_g, theta[-1]))

#    pdb.set_trace()
    plt.savefig('a1_robot_motion.png')
    plt.show()

if __name__=="__main__":
    main()
