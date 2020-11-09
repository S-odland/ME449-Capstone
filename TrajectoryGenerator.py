## Library imports

import modern_robotics as mr 
import numpy as np
import math as m 

## This file generates a reference trajectory for the end-effector frame {e}. The trajectory consists of 
## eight concatenated trahectory segments beginning and ending at rest.

## INPUTS:
## ## The initial configuration of the end-effecotr in the reference Trajectory (Tse,initial)
## ## The cube's initial configuration (Tsc,initial)
## ## The cube's desired final configuration (Tsc,final)
## ## The end-effector's configuration relative to the cube when it is grasping the cube (Tce,grasp)
## ## The end-effector's standoff confuration above the sube, before and after grasping, relative to the cube (Tce,standoff)
## --------> Specifies the configuration of the end-effector {e} relative to the cube frame {c} before lowering to the grasp
## --------> configuration Tce,grasp
##
## OUTPUTS:
## ## A representation of the N configuration of the end-effecotr along the entire concatenated eight-segment reference trajectory.
## --------> Each N reference point represent a transformation matrix Tse of the end effector frame relative to {s} at an instant
## --------> in time, plus the grupper state(0,1)
## ## A csv file with th eeinter eight-segment reference trajectory
## --------> each line of the csv corresponds to one configuration Tse 
## --------> [r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper state]

## This code should use either mr.ScrewTrajectory or mr.CartesianTrajectory to generate individual trajectory segments

## A reference trajectory consists of eight segments:
##    1. A trajectory to move the gripper from its initial configuration to a "standoff" configuration a few cm above the block
##    2. A trajectory to move the gripper down to the grasp position
##    3. Closing of the gripper
##    4. A trajectory to move the gripper back to the "standoff" configuration 
##    5. A trajectpry to move the gripper to a "standoff" confugration above the final configuration
##    6. A trajectory to move the gripper tothe final configuration of the object
##    7. Opening of the gripper
##    8. A trajectory to move the gripper back to the "standoff" configuration

Tsc_initial = np.array([[1,0,0,1],[0,1,1,0],[0,0,1,0.025],[0,0,0,2]]) # initial configuration of cube
Tsc_final = np.array([[0,1,0,0],[-1,0,0,-1],[0,0,1,0.025],[0,0,0,1]]) # final configuration of cube

Tse_initial = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0.5],[0,0,0,1]]) # initial configuration of the gripper
Tce_standoff = np.array([[-0.7071,0,-0.7071,1],[0,1,0,0],[-0.7071,0,-0.7071,0.1],[0,0,0,1]])
Tce_grip = np.array([[-0.7071,0,-0.7071,1],[0,1,0,0],[-0.7071,0,-0.7071,0.025],[0,0,0,1]])

grip_state = 0

