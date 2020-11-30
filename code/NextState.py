## Library imports

import modern_robotics as mr 
import numpy as np
import math as m 
import pandas as pd
import itertools

## INPUTS ##
## 1. A 12-vector representing the current configuration of the robot (3 variables for the 
##    chassis configuration, 5 variables for the arm configuration and 4 variables for the
##    wheel angles)
## 2. A 9-vector of controls indicating the arm joint speeds theta_dot (5 variables) and
##    and the wheels speeds u (4 variables)
## 3. A timestep delta_t
## 4. A positive real value indicating the maximum angular speed of the arm joints and 
##    the wheels (recommended speed limit of 5)

## OUTPUTS ##
## 1. A 12-vector representating the configuration of the robot time delta_t later

## NextState should be based on a simple first order Euler step:
## - new arm joints = (old arm joints) + (joint speeds) * delta_t
## - new wheel angles = (old wheel angles) + (wheel speeds) * delta_t
## - new chassis configuration is obtained from odometry (ch. 13.4)
def writeCSV(line):
    data = pd.DataFrame(line)
    data.to_csv("nextstate.csv",header=False,index=False)

def NextState(curConfig,controls,del_t,limits):
    nextState = []
    curJointConfig = np.array(curConfig[3:8])
    q_cur= np.array(curConfig[0:3])
    curWheelConfig = np.array(curConfig[8:])
    jointSpeeds = np.array(controls[0:5])
    u = np.array(controls[5:])

    for i in range(len(controls)): 
        if controls[i] > limits: controls[i] = limits
        elif controls[i] < -limits: controls[i] = -limits
    #print(controls)

    nextJointConfig = curJointConfig + jointSpeeds*del_t
    nextWheelConfig = curWheelConfig + u*del_t

    ## YouBot Properties:
    l = 0.47/2
    w = 0.3/2
    r = 0.0475
    phi_k = q_cur[0]

    F = (r/4)*np.array([[-1/(l+w), 1/(l+w), 1/(l+w),-1/(l+w)],[1, 1, 1, 1],[-1, 1, -1, 1]])
    del_theta = u*del_t
    Vb = np.dot(F,del_theta)

    Vb6 = np.array([0,0,Vb[0],Vb[1],Vb[2],0])
    [R,p] = mr.TransToRp(mr.MatrixExp6(mr.VecTose3(Vb6)))
    omg = mr.so3ToVec(mr.MatrixLog3(R))

    del_x = p[0]
    del_y = p[1]
    del_w = omg[2]

    if del_w == 0: del_qb = np.array([del_w,del_x,del_y])
    else: del_qb = np.array([del_w, \
                      (del_x*m.sin(del_w) + del_y*(m.cos(del_w)-1))/del_w, \
                      (del_y*m.sin(del_w) + del_y*(1-m.cos(del_w)))/del_w])

    del_q = np.dot(np.array([[1,0,0],[0,m.cos(phi_k),-m.sin(phi_k)],[0,m.sin(phi_k),m.cos(phi_k)]]), \
                   del_qb)
    q_next = q_cur + del_q

    nextConfig = [list(q_next),list(nextJointConfig),list(nextWheelConfig)]
    nextState = list(itertools.chain(*nextConfig))

    return nextState

def simControls(curConfig,controls,del_t,limits):
    robotConfigs = list(np.zeros(int(1/del_t)))
    for i in range(0,int(1/del_t)):
        robotConfigs[i] = curConfig
        curConfig = NextState(curConfig,controls,del_t,limits)

    for i in range(len(robotConfigs)):
        robotConfigs[i].append(0)
       
    return robotConfigs

if __name__ == '__main__':
    del_t = 0.01
    limits = 10
    controls = [0,0,0,0,0,-10,10,-10,10]
    curConfig = [0,0,0,0,0,0,0,0,0,0,0,0]
    line = simControls(curConfig,controls,del_t,limits)
    writeCSV(line)
        
