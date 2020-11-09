## Library imports

import modern_robotics as mr 
import numpy as np

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