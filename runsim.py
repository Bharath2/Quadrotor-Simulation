import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as Axes3D

from PathPlanning import RRTStar, Map
from TrajGen import trajGenerator, Helix_waypoints, Circle_waypoints
from Quadrotor import QuadSim
import controller
np.random.seed(8)

# 3D boxes   lx, ly, lz, hx, hy, hz
obstacles = [[-5, 25, 0, 20, 35, 60],
             [30, 25, 0, 55, 35, 100],
             [45, 35, 0, 55, 60, 60],
             [45, 75, 0, 55, 85, 100],
             [-5, 65, 0, 30, 70, 100],
             [70, 50, 0, 80, 80, 100]]

# limits on map dimensions
bounds = np.array([0,100])
# create map with obstacles
mapobs = Map(obstacles, bounds, dim = 3)

#plan a path from start to goal
start = np.array([80,20,10])
goal = np.array([30,80,80])

rrt = RRTStar(start = start, goal = goal,
              Map = mapobs, max_iter = 500,
              goal_sample_rate = 0.1)

waypoints, min_cost = rrt.plan()


#scale the waypoints to real dimensions
waypoints = 0.02*waypoints

#Generate trajectory through waypoints
traj = trajGenerator(waypoints, max_vel = 10, gamma = 1e6)

#initialise simulation with given controller and trajectory
Tmax = traj.TS[-1]
des_state = traj.get_des_state
sim = QuadSim(controller,des_state,Tmax)

#create a figure
fig = plt.figure()
ax = Axes3D.Axes3D(fig)
ax.set_xlim((0,2))
ax.set_ylim((0,2))
ax.set_zlim((0,2))

#plot the waypoints and obstacles
rrt.draw_path(ax, waypoints)
mapobs.plotobs(ax, scale = 0.02)

#run simulation
sim.run(ax)
