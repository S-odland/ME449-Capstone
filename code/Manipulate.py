import modern_robotics as mr 
import numpy as np
import math as m 
import pandas as pd
import itertools
import TrajectoryGenerator as TG
import NextState as NS
import FeedforwardControl as FC
import matplotlib

def writeCSV(line):
    data = pd.DataFrame(line)
    data.to_csv("manipulate.csv",header=False,index=False)

def getRefTraj():
    eOffset = 0.075
    Tsc_initial = np.array([[1,0,0,1],[0,1,0,0],[0,0,1,0.025],[0,0,0,1]]) # initial configuration of cube
    Tsc_final = np.array([[0,1,0,0],[-1,0,0,-1],[0,0,1,0.025],[0,0,0,1]]) # final configuration of cube
    Tse_initial = np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]]) # initial configuration of the gripper
    Tce_standoff = np.array([[-0.7071,0,-0.7071,0],[0,1,0,0],[0.7071,0,-0.7071,eOffset],[0,0,0,1]]) # standoff configuration 
    Tce_grip = np.array([[-0.7071,0,-0.7071,0],[0,1,0,0],[0.7071,0,-0.7071,0],[0,0,0,1]])
    refTraj,grip_states = TG.TrajectoryGenerator(Tsc_initial,Tsc_final,Tse_initial,Tce_standoff,Tce_grip)
    
    return refTraj,grip_states    

def getCurRef(refTraj,i):
    Xd = refTraj[i]
    Xdn = refTraj[i+1]
    
    return Xd,Xdn

def timeStep(curConfig,del_t,limits,controls,kp,ki,Xerr_int,i,trajectories):
    X,theta_cur,F = FC.getActConfig(curConfig,controls,del_t,limits)
    Kp = kp*np.identity(6)
    Ki = ki*np.identity(6)
    Xd,Xdn = getCurRef(trajectories,i)
    V,Xerr,Xerr_int = FC.FeedbackControl(X,Xd,Xdn,Kp,Ki,del_t,Xerr_int)
    pJe = FC.getPsuedo(F,theta_cur)
    controls = np.dot(pJe,V)
    curConfig,F = NS.NextState(curConfig,controls,del_t,limits)
    return curConfig,controls,Xerr,Xerr_int

def main():
    trajectories,grip_states = getRefTraj()
    grip_states = list(grip_states)
    del_t = 0.01
    limits = 10
    kp,ki = [0,0]
    robotConfigs = list(np.zeros(len(trajectories)-1))
    robotControls = list(np.zeros(len(trajectories)-1))
    robotErr = list(np.zeros(len(trajectories)-1))

    ## configuration format: [phi,x,y,j1,j2,j3,j4,j5,w1,w2,w3,w4,gripper state]
    curConfig = [m.pi/3,0.2,0,0,0,0,0,0,0,0,0,0]
    controls = np.array([0,0,0,0,0,0,0,0,0])
    Xerr = np.array([0,0,0,0,0,0])
    Xerr_int = np.array([0,0,0,0,0,0])

    print('Generating animation CSV file')
    for i in range(len(trajectories)-1):
        robotConfigs[i] = curConfig
        robotControls[i] = controls 
        robotErr[i] = Xerr
        curConfig,controls,Xerr,Xerr_int = timeStep(curConfig,del_t,limits,controls,kp,ki,Xerr_int,i,trajectories)

    for i in range(len(robotConfigs)):
        robotConfigs[i].append(grip_states[i])
    writeCSV(robotConfigs)

if __name__ == '__main__':
    main()