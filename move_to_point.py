#!/usr/bin/env python

from collections import namedtuple
import matplotlib.pyplot as plt
from matplotlib.patches import Arrow, Circle
from matplotlib.collections import PatchCollection
import numpy as np
import pdb

# Computer Science 685 Assignment 3
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

    def move_to_point(self, x_g, y_g):
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

    def move_to_line(self, l, v, k_d, k_h):
        time_delta = .8
        x_plot = []
        y_plot = []
        theta_plot = []
        a, b, c = l
        line_orientation = np.arctan2(-a, b)
        
        while True:
            pose = self.pose
            x_plot.append(pose.x)
            y_plot.append(pose.y)
            theta_plot.append(pose.theta)

            if pose.x <= 0.0 or pose.x > 50.0:
                break
            
            # If the robot is beneath the line the distance should be
            # negative, so get rid of absolute value.
            distance = pose.x * a + pose.y * b + c/np.sqrt(a**2 + b**2)
            omega = -k_d * distance + k_h * (line_orientation - pose.theta)
            print("x={:<6.4} y={:<6.4} distance={:<6.4} theta={:<6.4} v={:<6.4} omega={:<6.4} line_orientation={:<6.4}".format(pose.x,pose.y,distance, pose.theta,v,omega,line_orientation))

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
    ax.set_xlim(min_x - 20, min_x + rng + 20)
    ax.set_ylim(min_y - 20, min_y + rng + 20)
    ax.set_aspect('equal', 'datalim')

    start_arrow = Arrow(start.x, start.y, 10*np.cos(start.theta), 10*np.sin(start.theta), 2.0)
    goal_arrow = Arrow(goal.x, goal.y, 10*np.cos(goal.theta), 10*np.sin(goal.theta), 2.0)
    ax.add_collection(PatchCollection([start_arrow, goal_arrow]))

def do_move_to_point():
    x_g = 0.0
    y_g = 0.0


    for initial_pose in [Pose(0,    50, deg2rad(180)),
                         Pose(50 * np.cos(np.pi/4), 50 * np.sin(np.pi/4), deg2rad(135)),
                         Pose(50,    0,  deg2rad(270)),
                         Pose(-50 * np.cos(np.pi/4), -50 * np.sin(np.pi/4), deg2rad(135)),
                         Pose(-50,   0,  deg2rad(180)),
                         ]:
        robot = DifferentialDriveRobot(pose=initial_pose)
        x,y, theta = robot.move_to_point(x_g, y_g)
        plot(x, y, initial_pose, Pose(x_g, y_g, theta[-1]))

    plt.savefig('a3_move_to_point.png')
    plt.show()

def do_move_to_line():
    # y = ax + c
    # -ax + y - c = 0
    a = 1.0
    b = 1.0
    c = 0.0
    l = (-a, b, -c)

#    pdb.set_trace()
    plt.scatter([i for i in range(50)], [a * i + c for i in range(50)])

    for initial_pose in [
           # Pose(10.0, 10.0, deg2rad(180)),
           # Pose(5.0, 10.0, deg2rad(180)),
           # Pose(1.0, 5.0, deg2rad(180)),
           # Pose(20.0, 20.0, deg2rad(90)),
           # Pose(20.0, 30.0, deg2rad(90)),
           # Pose(20.0, 40.0, deg2rad(270)),
            Pose(10.0, 43.0, deg2rad(180)),
            Pose(10.0, 40.0, deg2rad(180)),
           # Pose(10.0, 5.0, deg2rad(0)),
           # Pose(6.0, 3.0, deg2rad(0)),
           # Pose(30.0, 10.0, deg2rad(180)),
           # Pose(20.0, 10.0, deg2rad(45)),
           # Pose(2.0, 7.0, deg2rad(0)),
            ]:
        robot = DifferentialDriveRobot(pose=initial_pose)
        x,y, theta = robot.move_to_line(l, 1.0, 0.1, 1.0)
        plot(x, y, initial_pose, Pose(x[-1], y[-1], theta[-1]))

    plt.savefig('a3_move_line.png')
    plt.show()

def main():
    do_move_to_line()

if __name__=="__main__":
    main()
    
