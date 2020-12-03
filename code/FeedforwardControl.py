## Library Imports

import modern_robotics as mr 
import numpy as np
import math as m 
import pandas as pd
import itertools
import TrajectoryGenerator as TG
import NextState as NS

## V(t) = [Adx-1xd]Vd(t) + KpXerr(t) + Ki [Xerr(t)dt]0-->t

## INPUTS ##
## 1. The current actual end-effector configuration X (Tse)
## 2. The current end-effector reference configuration Xd (Tse,d)
## 3. The end-effector reference configuration at the next timestep in the reference
##    trajectory, Xd,next(Tse,d,next) at a timestep delta_t later
## 4. The PI gain matrices Kp and Ki
## 5. The timestep delta_t between reference trajectory configurations

## OUTPUT ##
## The commanded end-effector twist V expressed in the end-effector frame {e}

def getConsts():
    B1 = np.array([0,0,1,0,0.033,0])
    B2 = np.array([0,-1,0,-0.5076,0,0])
    B3 = np.array([0,-1,0,-0.3526,0,0])
    B4 = np.array([0,-1,0,-0.2176,0,0])
    B5 = np.array([0,0,1,0,0,0])

    Blist = np.array([B1,B2,B3,B4,B5])
    T0e_init = np.array([[1,0,0,0.0330],[0,1,0,0],[0,0,1,0.6546],[0,0,0,1]])
    Tb0 = np.array([[1,0,0,0.1662],[0,1,0,0],[0,0,1,0.0026],[0,0,0,1]])

    return Blist.T,T0e_init,Tb0

def writeCSV(line):
    data = pd.DataFrame(line)
    data.to_csv("nextstate.csv",header=False,index=False)

def getActConfig(curConfig,controls,del_t,limits):
    l = 0.47/2
    w = 0.3/2
    r = 0.0475
    theta_cur = np.array(curConfig[3:8])
    print('theta:',theta_cur)
    phi,x,y = np.array(curConfig[0:3])
    F = (r/4)*np.array([[-1/(l+w), 1/(l+w), 1/(l+w),-1/(l+w)],[1, 1, 1, 1],[-1, 1, -1, 1]])
    Blist,M,Tb0 = getConsts()

    T0e = mr.FKinBody(M,Blist,theta_cur)
    Tsb = np.array([[m.cos(phi),-m.sin(phi),0,x],\
                    [m.sin(phi),m.cos(phi),0,y],\
                    [0,0,1,0.0963],\
                    [0,0,0,1]])

    X = np.dot(Tsb,np.dot(Tb0,T0e))
    return X,theta_cur,F

def getRefTraj():
    eOffset = 0.075
    Tsc_initial = np.array([[1,0,0,1],[0,1,0,0],[0,0,1,0.025],[0,0,0,1]]) # initial configuration of cube
    Tsc_final = np.array([[0,1,0,0],[-1,0,0,-1],[0,0,1,0.025],[0,0,0,1]]) # final configuration of cube
    Tse_initial = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0.5],[0,0,0,1]]) # initial configuration of the gripper
    Tce_standoff = np.array([[-0.7071,0,-0.7071,0],[0,1,0,0],[0.7071,0,-0.7071,eOffset],[0,0,0,1]]) # standoff configuration 
    Tce_grip = np.array([[-0.7071,0,-0.7071,0],[0,1,0,0],[0.7071,0,-0.7071,0],[0,0,0,1]])
    refTraj = TG.TrajectoryGenerator(Tsc_initial,Tsc_final,Tse_initial,Tce_standoff,Tce_grip)
    return refTraj    

def getCurRef(refTraj,i):
    Xd = refTraj[i]
    Xdn = refTraj[i+1]
    return Xd,Xdn

def getPsuedo(F,theta_cur):
    Blist,M,Tb0 = getConsts()
    T0e = mr.FKinBody(M,Blist,theta_cur)

    Ja = mr.JacobianBody(Blist,theta_cur)
    F6 = np.vstack([np.zeros(len(F[0])),np.zeros(len(F[0])),F[0],F[1],F[2],np.zeros(len(F[0]))])
    Jb = np.dot(np.dot(mr.Adjoint(np.linalg.inv(T0e)), \
                       mr.Adjoint(np.linalg.inv(Tb0))),\
                F6)
    
    Je = np.c_[Jb,Ja]
    print('Je:',Je)
    pJe = np.dot(Je.T,np.linalg.inv(np.dot(Je,Je.T)))
    
    return pJe

def FeedbackControl(X,Xd,Xdn,Kp,Ki,del_t,Xerr_int):

    Xerr = mr.se3ToVec(mr.MatrixLog6(np.dot(np.linalg.inv(X),Xd)))
    Xerr_int = np.add(Xerr_int,Xerr*del_t,out=Xerr_int,casting="unsafe")

    Vd = mr.se3ToVec((1/del_t)*mr.MatrixLog6(np.dot(np.linalg.inv(Xd),Xdn)))
    print('Vd:',Vd)
    Adxxd = np.dot(mr.Adjoint(np.linalg.inv(X)),mr.Adjoint(Xd))
    print('Adxxd.Vd:',np.dot(Adxxd,Vd))
    print('Xerr:',Xerr)

    V = np.dot(Adxxd,Vd) + np.dot(Kp,Xerr) + np.dot(Ki,Xerr_int)

    return V
    
if __name__ == '__main__':
    ## when testing this, will probably remove init function and just do that code here in the executable

    del_t = 0.01
    limits = 10
    controls = [0,0,0,0,0,0,0,0,0]
    curConfig = [0,0,0,0,0,0.2,-1.6,0,0,0,0,0]

    X,theta_cur,F = getActConfig(curConfig,controls,del_t,limits)
    kp,ki = [0,0]

    Kp = kp*np.identity(6)
    Ki = ki*np.identity(6)
    Xerr_int = np.array([0,0,0,0,0,0])
    print('X:',X)

    trajectories = getRefTraj()
    #Xd,Xdn = getCurRef(trajectories,0)
    Xd = np.array([[0,0,1,0.5],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]])
    Xdn = np.array([[0,0,1,0.6],[0,1,0,0],[-1,0,0,0.3],[0,0,0,1]])
    V = FeedbackControl(X,Xd,Xdn,Kp,Ki,del_t,Xerr_int)
    print('V:',V)
    pJe = getPsuedo(F,theta_cur)
    controls = np.dot(pJe,V)
    print('Controls:',controls)
    curConfig = NS.NextState(curConfig,controls,del_t,limits)

    
        
    
