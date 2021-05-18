import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as Axes3D
from .rrtutils import *

class RRT:

    def __init__(self, start, goal, Map,
                 max_extend_length = 5.0,
                 path_resolution = 0.5,
                 goal_sample_rate = 0.05,
                 max_iter = 100 ):
        self.start = Node(start)
        self.goal = Node(goal)
        self.max_extend_length = max_extend_length
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.dim = start.shape[0]
        self.tree = Rtree(self.dim)
        self.map = Map

    def plan(self):
        """Plans the path from start to goal while avoiding obstacles"""
        self.tree.add(self.start)
        for i in range(self.max_iter):
            #Generate a random node (rnd_node)
            rnd_node = self.get_random_node()
            #Get nearest node (nearest_node)
            nearest_node = self.tree.nearest(rnd_node)
            #Get new node (new_node) by connecting
            new_node = self.steer(nearest_node,rnd_node)
            #If the path between new_node and the nearest node is not in collision
            if not self.map.collision(nearest_node.p,new_node.p):
              self.tree.add(new_node)
              # If the new_node is very close to the goal, connect it
              # directly to the goal and return the final path
              if self.dist(new_node,self.goal) <= self.max_extend_length:
                  if not self.map.collision(new_node.p,self.goal.p):
                      self.goal.parent = new_node
                      return self.final_path()
        # cannot find path
        return None

    @staticmethod
    def dist(from_node, to_node):
        #euler distance
        return np.linalg.norm(from_node.p - to_node.p)

    def steer(self,from_node, to_node):
        """Connects from_node to a new_node in the direction of to_node
        with maximum distance max_extend_length
        """
        dist = self.dist(from_node, to_node)
        #Rescale the path to the maximum extend_length
        if dist > self.max_extend_length:
            diff = from_node.p - to_node.p
            to_node.p  = from_node.p - diff/dist * self.max_extend_length
        to_node.parent = from_node
        return to_node

    def sample(self):
        # Sample random point inside boundaries
        lower,upper = self.map.bounds
        return lower + np.random.rand(self.dim)*(upper - lower)

    def get_random_node(self):
        """Sample random node inside bounds or sample goal point"""
        if np.random.rand() > self.goal_sample_rate:
            rnd = self.sample()
        else:
            rnd = self.goal.p
        return Node(rnd)

    def final_path(self):
        """Compute the final path from the goal node to the start node"""
        path = []
        node = self.goal
        if (node.p == node.parent.p).all(): node = node.parent
        while node.parent:
          path.append(node.p)
          node = node.parent
        path.append(self.start.p)
        return np.array(path[::-1])

    def draw_graph(self,ax):
        '''plot the whole graph'''
        for node in self.tree.all():
            if node.parent:
                xy = np.c_[node.p,node.parent.p]
                ax.plot(*xy, "-g",zorder = 5)

    def draw_path(self,ax,path):
        '''draw the path if available'''
        if path is None:
            print("path not available")
        else:
            ax.plot(*np.array(path).T, '-', color = (0.9, 0.2, 0.5, 0.8), zorder = 5)

    def draw_scene(self,path = None,ax = None):
        '''draw the whole scene'''
        if ax is None:
            fig = plt.figure()
            if self.dim == 3:
                ax = Axes3D.Axes3D(fig)
            elif self.dim == 2:
                ax = plt.axes()
            else:
                print('cannot plot for current dimensions')
                return
        self.draw_graph(ax)
        self.draw_path(ax,path)
        self.map.plotobs(ax)
        plt.show()
