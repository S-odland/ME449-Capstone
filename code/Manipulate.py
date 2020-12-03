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
    X,theta_cur,F = FC.getActConfig(curConfig)
    Xd,Xdn = getCurRef(trajectories,i)
    V,Xerr,Xerr_int = FC.FeedbackControl(X,Xd,Xdn,Kp,Ki,del_t,Xerr_int)
    pJe = FC.getPsuedo(F,theta_cur)
    controls = np.dot(pJe,V)
    controls = arrangeControls(controls)
    curConfig = NS.NextState(curConfig,controls,del_t,limits)
    return curConfig,controls,Xerr,Xerr_int,X

def main():
    trajectories,grip_states = getRefTraj()
    grip_states = list(grip_states)
    del_t = 0.01
    limits = 10
    kp,ki = [0,0]
    robotConfigs = list(np.zeros(len(trajectories)-1))
    robotControls = list(np.zeros(len(trajectories)-1))
    robotErr = list(np.zeros(len(trajectories)-1))
    endEfConfig = list(np.zeros(len(trajectories)-1))

    Kp = kp*np.identity(6)
    Ki = ki*np.identity(6)

    ## configuration format: [phi,x,y,j1,j2,j3,j4,j5,w1,w2,w3,w4,gripper state]
    curConfig = [0,0,0,0,0,0,0,0,0,0,0,0]
    controls = np.array([0,0,0,0,0,0,0,0,0])
    Xerr = np.array([0,0,0,0,0,0])
    Xerr_int = np.array([0,0,0,0,0,0])
    X = np.array([[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]])

    print('Generating animation CSV file')
    for i in range(len(trajectories)-1):
        robotConfigs[i] = curConfig
        robotControls[i] = controls 
        robotErr[i] = Xerr
        endEfConfig[i] = X
        curConfig,controls,Xerr,Xerr_int,X = timeStep(curConfig,del_t,limits,controls,Kp,Ki,Xerr_int,i,trajectories)
        

    for i in range(len(robotConfigs)):
        robotConfigs[i].append(grip_states[i])
    writeCSV(robotConfigs)
    writeCSVmat(endEfConfig,grip_states)
    print('Done.')

if __name__ == '__main__':
    main()