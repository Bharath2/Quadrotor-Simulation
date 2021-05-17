'''
  map utils
  obstacle map with collision checking

  author: Bharath Chandra
  email: iambharathchandra@gmail.com
'''

from rtree import index
import numpy as np
from matplotlib.pyplot import Rectangle

#obstacle map
class Map:
  def __init__(self,obstacle_list,bounds,path_resolution = 0.5,dim = 3):
    self.dim = dim
    self.idx = self.get_tree(obstacle_list,dim)
    self.len = len(obstacle_list)
    self.path_res = path_resolution
    self.obstacles = obstacle_list
    self.bounds = bounds

  @staticmethod
  def get_tree(obstacle_list,dim):
    '''initialise map with given obstacle_list'''
    p = index.Property()
    p.dimension = dim
    ls = [(i,(*obj,),None) for i, obj in enumerate(obstacle_list)]
    return index.Index(ls, properties=p)

  def add(self,obstacle):
    '''add new obstacle'''
    self.idx.insert(self.len,obstacle)
    self.obstacles.append(obstacle)
    self.len += 1

  def collision(self,start,end):
    '''find if the ray between start and end collides with obstacles'''
    dist = np.linalg.norm(start-end)
    n = int(dist/self.path_res)
    points = np.linspace(start,end,n)
    for p in points:
      if self.idx.count((*p,)) != 0 :
          return True
    return False

  def inbounds(self,p):
      '''Check if p lies inside map bounds'''
      lower,upper = self.bounds
      return (lower <= p).all() and (p <= upper).all()

  def plotobs(self,ax,scale = 1):
    '''plot all obstacles'''
    obstacles = scale*np.array(self.obstacles)
    if self.dim == 2:
        for box in obstacles:
            l = box[2] - box[0]
            w = box[3] - box[1]
            box_plt = Rectangle((box[0], box[1]),l,w,color='k',zorder = 1)
            ax.add_patch(box_plt)
    elif self.dim == 3:
        for box in obstacles:
            X, Y, Z = cuboid_data(box)
            ax.plot_surface(X, Y, Z, rstride=1, cstride=1,color=(0.1, 0.15, 0.3, 0.2),zorder = 1)
    else: print('can not plot for given dimensions')


#to plot obstacle surfaces
def cuboid_data(box):
    l = box[3] - box[0]
    w = box[4] - box[1]
    h = box[5] - box[2]
    x = [[0, l, l, 0, 0],
         [0, l, l, 0, 0],
         [0, l, l, 0, 0],
         [0, l, l, 0, 0]]
    y = [[0, 0, w, w, 0],
         [0, 0, w, w, 0],
         [0, 0, 0, 0, 0],
         [w, w, w, w, w]]
    z = [[0, 0, 0, 0, 0],
         [h, h, h, h, h],
         [0, 0, h, h, 0],
         [0, 0, h, h, 0]]
    return box[0] + np.array(x), box[1] + np.array(y), box[2] + np.array(z)
