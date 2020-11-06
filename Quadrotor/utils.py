"""
author: Peter Huang
email: hbd730@gmail.com
license: BSD
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np
from math import sin, cos, asin, atan2, sqrt

def vee(S):
    return np.array([-S[1,2],S[0,2],-S[0,1]])

def RotToRPY(R):
    phi = asin(R[1,2])
    theta = atan2(-R[0,2]/cos(phi),R[2,2]/cos(phi))
    psi = atan2(-R[1,0]/cos(phi),R[1,1]/cos(phi))
    return phi, theta, psi

def RPYToRot(phi, theta, psi):
    """
    phi, theta, psi = roll, pitch , yaw
    """
    return np.array([[cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta), -cos(phi)*sin(theta)],
                     [-cos(phi)*sin(psi), cos(phi)*cos(psi), sin(phi)],
                     [cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi), sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi), cos(phi)*cos(theta)]])

def RotToQuat(R):
    """
    ROTTOQUAT Converts a Rotation matrix into a Quaternion
    from the following website, deals with the case when tr<0
    http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
    takes in W_R_B rotation matrix
    """

    tr = R[0,0] + R[1,1] + R[2,2]
    if tr > 0:
        S = sqrt(tr+1.0) * 2 # S=4*qw
        qw = 0.25 * S
        qx = (R[2,1] - R[1,2]) / S
        qy = (R[0,2] - R[2,0]) / S
        qz = (R[1,0] - R[0,1]) / S
    elif (R[0,1] > R[1,1]) and (R[0,0] > R[2,2]):
        S = sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2 # S=4*qx
        qw = (R[2,1] - R[1,2]) / S
        qx = 0.25 * S
        qy = (R[0,1] + R[1,0]) / S
        qz = (R[0,2] + R[2,0]) / S
    elif R[1,1] > R[2,2]:
        S = sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2 # S=4*qy
        qw = (R[0,2] - R[2,0]) / S
        qx = (R[0,1] + R[1,0]) / S
        qy = 0.25 * S
        qz = (R[1,2] + R[2,1]) / S
    else:
        S = sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2 # S=4*qz
        qw = (R[1,0] - R[0,1]) / S
        qx = (R[0,2] + R[2,0]) / S
        qy = (R[1,2] + R[2,1]) / S
        qz = 0.25 * S

    q = np.sign(qw) * np.array([qw, qx, qy, qz])
    return q

def writeNpArrayToFile(data):
    with open('state.csv','a') as f:
        np.savetxt(f, data, newline=",", fmt='%.2f')
        f.write('\n')

def outputTraj(x,y,z):
    output = []
    output.append((x,y,z))
    with open('traj.out', 'w') as fp:
        fp.write('\n'.join('%s %s %s' % item for item in output))
