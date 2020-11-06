'''
  Sample utils
  Uniform Sampling within N-dimensional hypersphere, hyperellipsoid.
  InformedSampler for RRT*, BIT* to get random node given goal, start and current minimum cost

  author: Bharath Chandra
  email: iambharathchandra@gmail.com
'''

import numpy as np
from numpy import linalg as LA


def SampleUnitDisc():
    '''uniformly sample a 2D unit Disc'''
    r = np.random.uniform(0, 1)
    phi = np.random.uniform(0, 2*np.pi)
    x = r * np.cos(phi)
    y = r * np.sin(phi)
    return np.array([x,y])


def SampleUnitNBall(dim = 3,num = 1):
    '''
    uniformly sample a N-dimensional unit UnitBall
    Reference:
      Efficiently sampling vectors and coordinates from the n-sphere and n-ball
      http://compneuro.uwaterloo.ca/files/publications/voelker.2017.pdf

    Input:
        num - no. of samples
        dim - dimensions

    Output:
        uniformly sampled points within N-dimensional unit ball
    '''
    #Sample on a unit N+1 sphere
    u = np.random.normal(0, 1, (num, dim + 2))
    norm = LA.norm(u, axis = -1,keepdims = True)
    u = u/norm
    #The first N coordinates are uniform in a unit N ball
    if num == 1: return u[0,:dim]
    return u[:,:dim]


def SphereSampler(center, radius, num = 1):
    '''
    uniformly sample inside N-dimensional hypersphere
    Input:
        center - Center of sphere
        radius - Radius of Sphere
        num - no. of samples
    Output:
       uniformly sampled points inside the hypersphere
    '''
    dim = center.shape[0]
    xball = SampleUnitNBall(dim,num)
    return radius*xball + center


class EllipsoidSampler:
    '''
    uniformly sample within a N-dimensional Ellipsoid
    Reference:
      Informed RRT*: Optimal Sampling-based Path Planning Focused via Direct Sampling
      of an Admissible Ellipsoidal Heuristic https://arxiv.org/pdf/1404.2334.pdf
    '''
    def __init__(self,center,axes = [],rot = []):
        '''
        Input:
            center -  centre of the N-dimensional ellipsoid in the N-dimensional
            axes -  axes length across each dimension in ellipsoid frame
            rot - rotation matrix from ellipsoid frame to world frame
        Output:
            uniformly sampled points within the hyperellipsoid
        '''
        self.dim = center.shape[0]
        self.center = center
        self.rot = rot
        if len(rot) == 0: self.rot = np.eye(self.dim)
        if len(axes) == 0: axes = [1]*self.dim
        self.L = np.diag(axes)

    def sample(self,num = 1):
        xball = SampleUnitNBall(self.dim,num)
        #Transform points in UnitBall to ellipsoid
        xellip = (self.rot@self.L@xball.T).T + self.center
        return xellip


class InformedSampler:
    '''
    uniformly sample within a N-dimensional Prolate-Hyperspheroid for informed RRT*
    with goal and start as focal points
    Reference:
      Informed RRT*: Optimal Sampling-based Path Planning Focused via Direct Sampling
      of an Admissible Ellipsoidal Heuristic https://arxiv.org/pdf/1404.2334.pdf
    '''
    def __init__(self, goal, start):
        self.dim = goal.shape[0]
        self.cmin = LA.norm(goal - start)
        center = (goal + start)/2
        #rotation matrix from ellipsoid frame to world frame
        C = self.RotationToWorldFrame(goal, start)
        #initialise EllipsoidSampler
        self.ellipsampler = EllipsoidSampler(center,rot = C)

    def sample(self, cmax, num = 1):
        '''
        Input:
            cmax - current best cost
            num - no. of samples
        Output:
            uniformly sampled point within informed region
        '''
        #Hyperspheroid axes lengths
        r1 = cmax/2
        ri = np.sqrt(cmax**2 - self.cmin**2)/2
        axes = [r1]+ [ri]*(self.dim - 1)
        self.ellipsampler.L = np.diag(axes)
        #return sampled point
        return self.ellipsampler.sample(num)

    def RotationToWorldFrame(self, goal, start):
        '''
        Given two focal points goal and start in N-Dimensions
        Returns rotation matrix from the ellipsoid frame to the world frame
        '''
        #Transverse axis of the ellipsoid in the world frame
        E1 = (goal - start) / self.cmin
        #first basis vector of the world frame [1,0,0,...]
        W1 = [1]+[0]*(self.dim - 1)
        #outer product of E1 and W1
        M = np.outer(E1,W1)
        #SVD decomposition od outer product
        U, S, V = LA.svd(M)
        #Calculate the middle diagonal matrix
        middleM = np.eye(self.dim)
        middleM[-1,-1] = LA.det(U)*LA.det(V)
        #calculate the rotation matrix
        C = U@middleM@V.T
        return C
