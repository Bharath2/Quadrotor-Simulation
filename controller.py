import numpy as np
from math import sin, cos
from Quadrotor import params
from Quadrotor.utils import RPYToRot,vee

kv = np.array([10,10,10]);
kp = np.array([30,30,30]);

kR = np.array([5,5,5])
kW = np.array([0.05,0.05,0.1])

# Constants
m = params.mass
g = params.g
I = params.I

eI = np.zeros(3)

#Nonlinear Controller for trajectory tracking
def run(state, des_state):
    yaw_des = des_state.yaw
    dyaw_des = des_state.yawdot

    #current Rotation from body frame to world
    bRw = RPYToRot(*state.rot).T
    #normal vector to body frame
    ZB = bRw[:,2]

    #position and velocity errors
    ep = des_state.pos - state.pos
    ev = des_state.vel - state.vel

    #command acceleration
    commd_acc = kp*ep + kv*ev + des_state.acc
    # Desired Thrust vector
    F_des = m*commd_acc + m*g*np.array([0,0,1])
    # Desired Thrust in ZB direction
    U = F_des@ZB

    #current acceleration
    curr_acc = U*ZB/m - g*np.array([0,0,1])
    #acceleration error
    ea = curr_acc - des_state.acc

    #command jerk
    commd_jerk = kp*ev + kv*ea + des_state.jerk
    # derivative of desired Thrust vector
    dF_des = m*commd_jerk
    # derivative of desired Thrust vector in ZB direction
    dU = dF_des@ZB

    #desired direction of normal vector
    ZB_des = F_des/np.linalg.norm(F_des)

    Xc = np.array([cos(yaw_des),sin(yaw_des),0])
    ZB_Xc = np.cross(ZB_des,Xc)

    YB_des = ZB_Xc/np.linalg.norm(ZB_Xc)
    XB_des = np.cross(YB_des,ZB_des)

    #desired Rotation matrix
    R_des = np.c_[XB_des.T,YB_des.T,ZB_des.T]

    #Rotation error
    eR = 0.5*vee(bRw.T@R_des - R_des.T@bRw)

    #projection of angular velocity on xB − yB plane
    hw = (m*commd_jerk - dU*ZB_des)/U
    #desired angular velocity
    omega_des = np.array([-hw@YB_des,
                           hw@XB_des,
                           dyaw_des*ZB_des[2]])

    #angular velocity error
    eW = omega_des - state.omega

    #moment
    M = np.array([kR*eR + kW*eW]).T

    return U, M

#Controller for hover 
def run_hover(state, des_state, dt):
    yaw_des = des_state.yaw
    global eI

    #position and velocity errors
    ep = des_state.pos - state.pos
    ev = des_state.vel - state.vel
    eI += ep*dt

    #command acceleration
    commd_acc = kp*ep + kv*ev + kp*eI + des_state.acc

    #Desired Thrust in ZB direction
    U = m*commd_acc[2] + m*g

    #Rotation error
    phi_des = 1/g*(commd_acc[0]*sin(yaw_des) - commd_acc[1]*cos(yaw_des))
    theta_des = 1/g*(commd_acc[0]*cos(yaw_des) + commd_acc[1]*sin(yaw_des))
    eR = np.array([phi_des,theta_des,yaw_des]) - state.rot

    #desired angular velocity
    omega_des = np.array([0,0,0])
    #angular velocity error
    eW = omega_des - state.omega;

    #moment
    M = np.array([kR*eR + kW*eW]).T

    return U, M
