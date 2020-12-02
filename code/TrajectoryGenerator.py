## Library imports

import modern_robotics as mr 
import numpy as np
import math as m 

################## EXECUTING CODE DIRECTIONS ################
#### OPTION 1 - CUT AND PASTABLE COMMENT ####
## 1. Open up a python termial in the directory that you have this file downloaded to
## 2. type or copy and paste the following code:
##
## import TrajectoryGenerator as TG
## TG.TrajectoryGenerator()
##
## 3. That's it
##
#### OPTION 2 ####
## 1. Open this file in Visual Studio Code
## 2. Run it (I call the function TrajectoryGenerator at the end of the file)
## 3. That's it
#############################################################

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

## scene duration total 4+1+1+1+8+1+1+1 = 18 = N

# computes transformation matrices in the end effector frame
def scTose(Tsc,Tce):
    Tse = np.matmul(Tsc,Tce)
    return Tse

# initializes all the transformation matrices needed
def InitTG(Tsc_initial,Tsc_final,Tse_initial,Tce_standoff,Tce_grip):
    Tse_standoffInit = scTose(Tsc_initial,Tce_standoff)
    Tse_gripInit = scTose(Tsc_initial,Tce_grip)

    Tse_standoffFinal = scTose(Tsc_final,Tce_standoff)
    Tse_gripFinal = scTose(Tsc_final,Tce_grip)

    return Tse_initial,Tse_standoffInit,Tse_gripInit,Tse_standoffFinal,Tse_gripFinal

# the bulk of the code
def TrajectoryGenerator(Tsc_initial,Tsc_final,Tse_initial,Tce_standoff,Tce_grip):

# calls InitTG to assign these transformation matrices
    Tse_initial,Tse_standoffInit,Tse_gripInit,Tse_standoffFinal,Tse_gripFinal = InitTG(Tsc_initial,Tsc_final,Tse_initial,Tce_standoff,Tce_grip)

# Xstarts and Xends to iterate through as inputs to ScrewTrajectory
    traj_iter = np.array([Tse_initial,Tse_standoffInit,Tse_gripInit,Tse_gripInit,Tse_standoffInit,Tse_standoffFinal, \
                                                                Tse_gripFinal,Tse_gripFinal, Tse_standoffFinal])
    k = 1
    grip_states = 0
    t = np.array([4,1,1,1,8,1,1,1]) # durations for each segment of the total trajectory
    trajectories = []
    gState = 0
    
# iterating through the eight segments of the simulation
    for i in range(0,len(traj_iter)-1):

        Xstart = traj_iter[i]
        Xend = traj_iter[i+1]
        Tf = t[i]
        N = k*Tf/0.01

        if i == 2 or i == 6:
            gState = not gState

        traj = mr.ScrewTrajectory(Xstart,Xend,Tf,N,3)

        for i in range(0,len(traj)):

            if gState:
                grip_states = np.vstack([grip_states,1])
            else:
                grip_states = np.vstack([grip_states,0])

            trajectories.append(traj[i])

    return trajectories

def writeCSV(trajectories):
# creates the csv file for CoppeliaSim
    f = open("trajectory.csv", "w") 
    
    for i in range(0,len(trajectories)):

        T = trajectories[i]
        trajectory = "%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%d\n" % \
                                        (T[0][0],T[0][1],T[0][2],T[1][0],T[1][1],T[1][2],T[2][0],T[2][1],\
                                          T[2][2],T[0][3],T[1][3],T[2][3],grip_states[i])

        f.write(trajectory)
    f.close()

if __name__ == '__main__':
    eOffset = 0.075
    Tsc_initial = np.array([[1,0,0,1],[0,1,0,0],[0,0,1,0.025],[0,0,0,1]]) # initial configuration of cube
    Tsc_final = np.array([[0,1,0,0],[-1,0,0,-1],[0,0,1,0.025],[0,0,0,1]]) # final configuration of cube
    Tse_initial = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0.5],[0,0,0,1]]) # initial configuration of the gripper
    Tce_standoff = np.array([[-0.7071,0,-0.7071,0],[0,1,0,0],[0.7071,0,-0.7071,eOffset],[0,0,0,1]]) # standoff configuration 
    Tce_grip = np.array([[-0.7071,0,-0.7071,0],[0,1,0,0],[0.7071,0,-0.7071,0],[0,0,0,1]])
    trajectories = TrajectoryGenerator(Tsc_initial,Tsc_final,Tse_initial,Tce_standoff,Tce_grip)
    writeCSV(trajectories)

