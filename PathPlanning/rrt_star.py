from .rrt import *
from .sampleutils import InformedSampler
from math import ceil

class RRTStar(RRT):

    def __init__(self, start, goal, Map,
                 max_extend_length = 10.0,
                 path_resolution = 0.5,
                 goal_sample_rate = 0.05,
                 max_iter = 200 ):
        super().__init__(start, goal, Map, max_extend_length,
                         path_resolution, goal_sample_rate, max_iter)
        self.final_nodes = []
        self.Informedsampler = InformedSampler(goal, start)

    def plan(self):
        """Plans the path from start to goal while avoiding obstacles"""
        self.start.cost = 0
        self.tree.add(self.start)
        for i in range(self.max_iter):
            #Generate a random node (rnd_node)
            rnd = self.get_random_node()
            # Get nearest node
            nearest_node = self.tree.nearest(rnd)
            # Get new node by connecting rnd_node and nearest_node
            new_node = self.steer(nearest_node, rnd)
            # If path between new_node and nearest node is not in collision
            if not self.map.collision(nearest_node.p,new_node.p):
              #add the node to tree
              self.add(new_node)
        #Return path if it exists
        if not self.goal.parent: path = None
        else: path = self.final_path()
        return path, self.goal.cost

    def add(self,new_node):
        near_nodes = self.near_nodes(new_node)
        # Connect the new node to the best parent in near_inds
        self.choose_parent(new_node,near_nodes)
        #add the new_node to tree
        self.tree.add(new_node)
        # Rewire the nodes in the proximity of new_node if it improves their costs
        self.rewire(new_node,near_nodes)
        #check if it is in close proximity to the goal
        if self.dist(new_node,self.goal) <= self.max_extend_length:
          # Connection between node and goal needs to be collision free
          if not self.map.collision(self.goal.p,new_node.p):
            #add to final nodes if in goal region
            self.final_nodes.append(new_node)
        #set best final node and min_cost
        self.choose_parent(self.goal,self.final_nodes)

    def choose_parent(self, node, parents):
        """Set node.parent to the lowest resulting cost parent in parents and
           node.cost to the corresponding minimal cost
        """
        # Go through all near nodes and evaluate them as potential parent nodes
        for parent in parents:
          #checking whether a connection would result in a collision
          if not self.map.collision(node.p,parent.p):
            #evaluating the cost of the new_node if it had that near node as a parent
            cost = self.new_cost(parent, node)
            #picking the parent resulting in the lowest cost and updating the cost of the new_node to the minimum cost.
            if cost < node.cost:
              node.parent = parent
              node.cost = cost

    def rewire(self, new_node, near_nodes):
        """Rewire near nodes to new_node if this will result in a lower cost"""
        #Go through all near nodes and check whether rewiring them to the new_node is useful
        for node in near_nodes:
          self.choose_parent(node,[new_node])
        self.propagate_cost_to_leaves(new_node)

    def near_nodes(self, node):
        """Find the nodes in close proximity to given node"""
        nnode = self.tree.len + 1
        r = ceil(5.5*np.log(nnode))
        return self.tree.k_nearest(node,r)

    def new_cost(self, from_node, to_node):
        """to_node's new cost if from_node were the parent"""
        return from_node.cost + self.dist(from_node, to_node)

    def propagate_cost_to_leaves(self, parent_node):
        """Recursively update the cost of the nodes"""
        for node in self.tree.all():
            if node.parent == parent_node:
                node.cost = self.new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)

    def sample(self):
        """Sample random node inside the informed region"""
        lower,upper = self.map.bounds
        if self.goal.parent:
          rnd = np.inf
          #sample until rnd is inside bounds of the map
          while not self.map.inbounds(rnd):
              # Sample random point inside ellipsoid
              rnd = self.Informedsampler.sample(self.goal.cost)
        else:
          # Sample random point inside boundaries
          rnd = lower + np.random.rand(self.dim)*(upper - lower)
        return rnd
