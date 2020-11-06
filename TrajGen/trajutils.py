import numpy as np
from collections import namedtuple

DesiredState = namedtuple('DesiredState', 'pos vel acc jerk yaw yawdot')

def polyder(t, k = 0, order = 10):
    if k == 'all':
        terms = np.array([polyder(t,k,order) for k in range(1,5)])
    else:
        terms = np.zeros(order)
        coeffs = np.polyder([1]*order,k)[::-1]
        pows = t**np.arange(0,order-k,1)
        terms[k:] = coeffs*pows
    return terms

def Hessian(T,order = 10,opt = 4):
    n = len(T)
    Q = np.zeros((order*n,order*n))
    for k in range(n):
        m = np.arange(0,opt,1)
        for i in range(order):
            for j in range(order):
                if i >= opt and j >= opt:
                    pow = i+j-2*opt+1
                    Q[order*k+i,order*k+j] = 2*np.prod((i-m)*(j-m))*T[k]**pow/pow
    return Q

def Circle_waypoints(n,Tmax = 2*np.pi):
    t = np.linspace(0,Tmax, n)
    x = 1+0.5*np.cos(t)
    y = 1+0.5*np.sin(t)
    z = 1+0*t
    return np.stack((x, y, z), axis=-1)

def Helix_waypoints(n,Tmax = 2*np.pi):

    t = np.linspace(0, Tmax, n)
    x = 1+0.5*np.cos(t)
    y = 1+0.5*np.sin(t)
    z = t/Tmax*2

    return np.stack((x, y, z), axis=-1)
