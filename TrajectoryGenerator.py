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

## scene duration total 4+1+1+1+8+1+1+1 = 18 = N

def configInCtoSE3(Tce_standoff,Tce_grip):

    Tse_standoff = np.matmul(Tsc_initial,Tce_standoff)
    Tse_grip = np.matmul(Tsc_initial,Tce_grip)

    return Tse_standoff,Tse_grip

def TrajectoryGenerator():
    
    eOffset = 0.075

    Tsc_initial = np.array([[1,0,0,1],[0,1,1,0],[0,0,1,0.025],[0,0,0,2]]) # initial configuration of cube
    Tsc_final = np.array([[0,1,0,0],[-1,0,0,-1],[0,0,1,0.025],[0,0,0,1]]) # final configuration of cube

    Tse_initial = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0.5],[0,0,0,1]]) # initial configuration of the gripper

    Tce_standoff = np.array([[-0.7071,0,-0.7071,0],[0,1,0,0],[0.7071,0,-0.7071,eOffset],[0,0,0,1]]) # standoff configuration 
    Tce_grip = np.array([[-0.7071,0,-0.7071,0],[0,1,0,0],[0.7071,0,-0.7071,0],[0,0,0,1]])

    grip_state = 0

    traj_states = np.array(Tse_initial.copy())
    grip_states = np.array([0])
    trajectories = np.array([Tse_initial[0][0],Tse_initial[0][1],Tse_initial[0][2],Tse_initial[1][0],Tse_initial[1][1],\
                            Tse_initial[1][2],Tse_initial[2][0],Tse_initial[2][1],Tse_initial[2][2],Tse_initial[0][3],\
                            Tse_initial[1][3],Tse_initial[2][3],grip_states[0]])

    Tse_standoffInit,Tse_gripInit = configInCtoSE3(Tce_standoff,Tce_grip)

    Tse_standoffFinal = np.array([[0,1,0,0],[0.7071,0,-0.7071,-1],[-0.7071,0,-0.7071,eOffset+0.025],[0,0,0,1]])
    Tse_gripFinal = np.array([[0,1,0,0],[0.7071,0,-0.7071,-1],[-0.7071,0,-0.7071,0.025],[0,0,0,1]])

    traj_iter = np.array([Tse_initial,Tse_standoffInit,Tse_gripInit,Tse_gripInit,Tse_standoffInit,Tse_standoffFinal, \
                                                                Tse_gripFinal,Tse_gripFinal, Tse_standoffFinal])
    
    t = np.array([4,1,1,1,8,1,1,1]) # durations for each segment of the total trajectory
    
    for i in range(0,len(traj_iter-1)):

        Xstart = traj_iter[i]
        Xend = traj_iter[i+1]
        Tf = t[i]
        N = Tf/0.01

        traj = mr.ScrewTrajectory(Xstart,Xend,Tf,N,3)

        for i in range(1,len(traj)):

            if i != 2 or i != 6:
                grip_states = np.vstack([grip_states,0])
            else:
                grip_states = np.vstack([grip_states,1])

        traj_states = np.vstack([traj_states,traj[1:len(traj)-1,:]])
    
    for i in range(1,len(traj_states)):

        T = traj_states[i]
        t = np.array([T[0][0],T[0][1],T[0][2],T[1][0],T[1][1],T[1][2],T[2][0],T[2][1],\
                       T[2][2],T[0][3],T[1][3],T[2][3],grip_states[i]])

        trajectories = np.vstack([trajectories,t])

    f = open("trajectory.csv", "w") 

    for i in range(len(trajectories)):
        trajectory = "%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%d\n" % \
                                                    (trajectories[i][0],trajectories[i][1],trajectories[i][2],trajectories[i][3],\
                                                    trajectories[i][4],trajectories[i][5],trajectories[i][6],trajectories[i][7],\
                                                    trajectories[i][8],trajectories[i][9],trajectories[i][10],trajectories[i][11],\
                                                    trajectories[i][12])
        f.write(trajectory)

    f.close()

