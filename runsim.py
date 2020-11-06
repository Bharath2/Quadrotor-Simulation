import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as Axes3D

from PathPlanning import RRTStar, Map
from TrajGen import trajGenerator, Helix_waypoints, Circle_waypoints
from Quadrotor import QuadSim
import controller

# create map with obstacles
obstacles = [[-5,25,0,20,35,60],
             [30,25,0,55,35,100],
             [45,35,0,55,60,60],
             [45,75,0,55,85,100],
             [-5,65,0,30,70,100],
             [70,50,0,80,80,100]]

bounds = np.array([0,100])
mapobs = Map(obstacles,bounds,dim = 3)

#plan the path
np.random.seed(7)
rrt = RRTStar(start=np.array([80,20,10]),
          goal=np.array((20,80,80)),
          Map=mapobs,max_iter=500,
          goal_sample_rate = 0.1)

waypoints, min_cost = rrt.plan()

#plot the map and the path
rrt.draw_scene(waypoints)

#scale the waypoints to real dimensions
scale = 0.2
waypoints = scale*waypoints

#Generate trajectory through waypoints
traj = trajGenerator(waypoints,max_vel = 10,gamma = 1000000)

#initialise simulation with given controller and trajectory
des_state = traj.get_des_state
sim = QuadSim(controller,des_state,Tmax = traj.TS[-1])

#plot the waypoints and obstacles
fig = plt.figure()
ax = Axes3D.Axes3D(fig)
ax.set_xlim((0,2))
ax.set_ylim((0,2))
ax.set_zlim((0,2))
rrt.draw_path(ax,waypoints)
mapobs.plotobs(ax,scale=scale)
#run simulation
sim.run(ax)
