## Library imports

import modern_robotics as mr 
import numpy as np
import math as m 

## INPUTS ##
## 1. A 12-vector representing the current configuration of the robot (3 variables for the 
##    chassis configuration, 5 variables for the arm configurationand 4 variables for the
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

def NextState(curConfig,controls,del_t,limits):
    nextState = []
    curJointConfig = np.array(curConfig[4:8])
    curChassisConfig = np.array(curConfig[0:3])
    curWheelConfig = np.array(curConfig[9:12])
    jointSpeeds = np.array(controls[0:5])
    wheelSpeeds = np.array(controls[6:9])

    for i, c in controls: 
        if c > limits: controls[i] = limits
        elif c < -limits: controls[i] = -limits

    nextJointConfig = curJointConfig + jointSpeeds*del_t
    nextWheelConfig = curWheelConfig + wheelSpeeds*del_t

    ## odometry - what do we know:
    ##     ## we have the 4 wheel angles
    ##     ## we have the 4 wheel speeds
    ##     ## we have the current chassis configuration

    ## YouBot Properties:
    l = 0.47/2
    w = 0.3/2
    r = 0.0475

    ## we have the u vector and r and the H(0) vector --> we can find wbz, vbx, vby
    ## u = H*Vb
    ## u = (1/r) * [[-l-w 1 -1][l+w 1 1][l+w 1 -1][-l-w 1 1]][wbz; vbx; vby]

    H = (1/r)*np.array([[-l-w, 1, -1],[l+w, 1, 1],[l+w, 1, -1],[-l-w, 1, 1]])
    Hinv = np.linalg.inv(H)

    Vb = Hinv*wheelSpeeds

    nextChassisConfig = curChassisConfig + Vb*del_t
    nextConfig = [list(nextChassisConfig),list(nextJointConfig),list(nextWheelConfig)]
    for i,n in nextConfig:
        nextState[i] = n

    return nextState

def simControls(curConfig,controls,del_t,limits):
    robotConfigs = curConfig
    for i in range(1/del_t):
        curConfig = NextState(curConfig,controls,del_t,limits)
        robotConfigs.append(curConfig)
    
        f = open("nextstate.csv", "w") 
    
    for i in range(0,len(robotConfigs)):
        T = robotConfigs[i]
        nextstate = "%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%d\n" % \
                                        (T[0][0],T[0][1],T[0][2],T[1][0],T[1][1],T[1][2],T[2][0],T[2][1],\
                                          T[2][2],T[0][3],T[1][3],T[2][3],0)
        f.write(nextstate)
    f.close()
    
        
