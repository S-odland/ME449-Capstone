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

    lim_range = [-limits,limits] ## max angular speeds of wheels and arm joints
    curJointConfig = curConfig[3:7]
    curChassisConfig = curConfig[0:2]
    curWheelConfig = curConfig[8:11]
    jointSpeeds = controls[0:4]
    wheelSpeeds = controls[5:8]

    for i, c in controls: 
        if c > limits: controls[i] = limits
        elif c < -limits: controls[i] = -limits

    nextJointConfig = curJointConfig + jointSpeeds*del_t
    nextWheelConfig = curWheelConfig + wheelSpeeds*del_t

    ## odometry - what do we know:
    ##     ## we have the 4 wheel angles
    ##     ## we have the 4 wheel speeds
    ##     ## we have the current chassis configuration

    