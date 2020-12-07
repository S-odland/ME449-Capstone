import modern_robotics as mr 
import numpy as np
import math as m 
import pandas as pd
import itertools
import TrajectoryGenerator as TG
import NextState as NS
import FeedforwardControl as FC
import matplotlib.pyplot as plt

## What to Simulate:
##
## Best, Overshoot, New Task
## Best - the best result of the final project, i.e. gains that produce the best
##        trajectory path (well tuned controller) - motion should be smooth with 
##        no overshoot and very little error
## Overshoot - a less well tuned controller that exhibits overshoot and a bit of 
##             oscillation - error should be eliminated before the end of trajectory
##             segment 1 -- likely to be feedforward+PI or just PI
## New Task - the best controls of new initial and end configurations of the cube
##
## In all cases, the initial confiugration of the end effector shoudl have at least
## 30 degrees of orientation error and 0.2 m of position error from the first config
## of the reference trajectory
##
## What else to hand in:
##
## A plot of the Xerr through the simulated trajectory (PDF or jpg format)
## A log file showing the program being called
## Xerr data file
## videos of the simulations described above
## .csv vile for for each of the simulations described above

## TODO: fix the initial configuration so that it doesnt automatically snap to the reference trajectory

def writeCSV(line):
    data = pd.DataFrame(line)
    data.to_csv("manipulate.csv",header=False,index=False)

def writeCSVmat(trajectories,grip_states):
    f = open("trajectorygen.csv", "w") 
    
    for i in range(0,len(trajectories)):

        T = trajectories[i]
        trajectory = "%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%d\n" % \
                                        (T[0][0],T[0][1],T[0][2],T[1][0],T[1][1],T[1][2],T[2][0],T[2][1],\
                                          T[2][2],T[0][3],T[1][3],T[2][3],grip_states[i])

        f.write(trajectory)
    f.close()

def getRefTraj():
    refTraj,grip_states = TG.TrajectoryGenerator()
    
    return refTraj,grip_states    

def getCurRef(refTraj,i):
    Xd = refTraj[i]
    Xdn = refTraj[i+1]
    
    return Xd,Xdn

def arrangeControls(con):
    one = np.array(con[4:])
    two = np.array(con[0:4])
    controls = np.hstack([one,two])
    return controls

def timeStep(curConfig,del_t,limits,controls,Kp,Ki,Xerr_int,i,trajectories):
    #err_joints = testJointLimits(curConfig,del_t,limits,controls,Kp,Ki,Xerr_int,i,trajectories)

    X,theta_cur,F = FC.getActConfig(curConfig)
    Xd,Xdn = getCurRef(trajectories,i)
    V,Xerr,Xerr_int = FC.FeedbackControl(X,Xd,Xdn,Kp,Ki,del_t,Xerr_int)

    pJe,Ja,Jb = FC.getPsuedo(F,theta_cur)
    # for i in range(5):
    #     if err_joints[i] == 1:
    #         Ja[:,i] = 0                   
    # pJe = np.linalg.pinv(np.c_[Jb,Ja],0.001)

    controls = np.dot(pJe,V)
    controls = arrangeControls(controls)
    curConfig = NS.NextState(curConfig,controls,del_t,limits)
    return curConfig,controls,Xerr,Xerr_int,X

def testJointLimits(curConfig,del_t,limits,controls,Kp,Ki,Xerr_int,i,trajectories):
    ## have to write out all these steps even though they are in timeStep to avoid recursion error
    X,theta_cur,F = FC.getActConfig(curConfig)
    Xd,Xdn = getCurRef(trajectories,i)
    V,Xerr,Xerr_int = FC.FeedbackControl(X,Xd,Xdn,Kp,Ki,del_t,Xerr_int)
    pJe,Ja,Jb = FC.getPsuedo(F,theta_cur)
    controls = np.dot(pJe,V)
    controls = arrangeControls(controls)
    curConfig = NS.NextState(curConfig,controls,del_t,limits)

    curJointConfig = np.array(curConfig[3:8])
    j1,j2,j3,j4,j5 = curJointConfig

    err_joints = [0,0,0,0,0]

    if j3 < -3.514:
        if j4 < 1: 
            err_joints[3] = 1
            err_joints[2] = 1
    elif  j2 + j3 > 1.853:
        if j4 > 0: 
            err_joints[3] = 1
            err_joints[2] = 1
    
    if j2 < -1.117 or j2 > 1.553:
        err_joints[1] = 1
    if j3 > 2.53 or j3 < -2.62:
        err_joints[2] = 1
    if j4 < -1.78 or j4 > 1.78:
        err_joints[3] = 1

    return err_joints

def plotErr(err):
    e = np.array([0,0,0,0,0,0])
    for i in range(len(err)):
        e = np.vstack([e,err[i]])
    e = e.T
    plt.plot(e[0,1:400],'r',e[1,1:400],'b',e[2,1:400],'g',e[3,1:400],'y',e[4,1:400],'m')
    plt.xlabel('Time')
    plt.ylabel('Joint Error (radians)')
    plt.title('Joint Error for Four Seconds')
    
    plt.show()

def main():
    trajectories,grip_states = getRefTraj()
    grip_states = list(grip_states)
    del_t = 0.01
    limits = 100
    kp,ki = [5,5]
    robotConfigs = list(np.zeros(len(trajectories)-1))
    robotControls = list(np.zeros(len(trajectories)-1))
    robotErr = list(np.zeros(len(trajectories)-1))
    endEfConfig = list(np.zeros(len(trajectories)-1))

    Kp = kp*np.identity(6)
    Ki = ki*np.identity(6)

    ## configuration format: [phi,x,y,j1,j2,j3,j4,j5,w1,w2,w3,w4,gripper state]
    ## adjust this config for 30 degrees error and 0.2 positional error

    curConfig = [0,0,0,0.6,1.128,-0.598,-2.101,-0.3,0,0,0,0] ## initial configuration found through inverse kinematics (newton-raphsom method)
    controls = np.array([0,0,0,0,0,0,0,0,0])
    Xerr_int = np.array([0,0,0,0,0,0])

    print('Generating animation CSV file')
    for i in range(len(trajectories)-1):
        robotConfigs[i] = curConfig
        curConfig,controls,Xerr,Xerr_int,X = timeStep(curConfig,del_t,limits,controls,Kp,Ki,Xerr_int,i,trajectories)
        robotControls[i] = controls 
        robotErr[i] = Xerr
        endEfConfig[i] = X

    for i in range(len(robotConfigs)):
        robotConfigs[i].append(grip_states[i])
    writeCSV(robotConfigs)
    print('Plotting Joint Error')
    #print(robotErr[:,0])
    plotErr(robotErr)
    print('Done')

if __name__ == '__main__':
    main()

