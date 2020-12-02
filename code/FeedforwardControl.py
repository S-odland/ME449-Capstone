## Library Imports

import modern_robotics as mr 
import numpy as np
import math as m 
import pandas as pd
import itertools
import importlib
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

    return Blist,T0e_init,Tb0

def writeCSV(line):
    data = pd.DataFrame(line)
    data.to_csv("nextstate.csv",header=False,index=False)

def getActConfig(curConfig,controls,del_t,limits):
    actConfig,F = NS.NextState(curConfig,controls,del_t,limits)
    theta_cur = np.array(actConfig[3:8])
    phi,x,y = np.array(actConfig[0:3])

    Blist,M,Tb0 = getConsts()

    T0e = mr.FKinBody(M,Blist,theta_cur)
    Tsb = np.array([[m.cos(phi),-m.sin(phi),0,x],[m.sin(phi),m.cos(phi),0,y],[0,0,1,0.0963],[0,0,0,1]])

    X = np.dot(Tsb,np.dot(Tb0,T0e))
    return X

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

def init(kp,ki):
    del_t = 0.01
    limits = 10
    controls = [0,0,0,0,0,0,0,0,0]
    curConfig = [0,0,0,0,0,0,0,0,0,0,0,0]

    X = getActConfig(curConfig,controls,del_t,limits)
    Xd,Xdn = getCurRef(getRefTraj(),0)

    Kp = kp*np.identity(5)
    Ki = ki*np.identity(5)

    Xerr_int_init = np.array([0,0,0,0,0,0])

    return X,Xd,Xdn,Kp,Ki,del_t,Xerr_int_init

def getPsuedo(curConfig,controls,del_t,limits):
    Blist,M,Tb0 = getConsts()
    actConfig,F = NS.NextState(curConfig,controls,del_t,limits)
    theta_cur = np.array(actConfig[3:8])
    T0e = mr.FKinBody(M,Blist,theta_cur)

    Ja = mr.JacobianBody(Blist,theta_cur)
    F6 = np.array([0,0,F[0],F[1],F[2],0])
    Jb = np.dot(np.dot(mr.Adjoint(np.linalg.inv(T0e)), \
                       mr.Adjoint(np.linalg.inv(Tb0))),\
                F6)
    
    Je = np.hstack(Jb,Ja)
    pJe = np.dot(Je.T,np.linalg.inv(np.dot(Je,Je.T)))
    
    return pJe

def FeedbackControl(X,Xd,Xdn,Kp,Ki,del_t,Xerr_int):

    Xerr = mr.se3ToVec(m.log10(np.dot(np.linalg.inv(X),Xd)))
    Xerr_int += Xerr*del_t

    Vd = (1/del_t)*m.log10(np.linalg.inv(Xd),Xdn)
    Adxxd = np.dot(mr.Adjoint(np.linalg.inv(X)),mr.Adjoint(Xd))

    V = np.dot(Adxxd,Vd) + np.dot(Kp,Xerr) + np.dot(Ki,Xerr_int)

    return V

def calcControls(X,Xd,Xdn,Kp,Ki,del_t,Xerr_int,curConfig,controls,limits):
    V = FeedbackControl(X,Xd,Xdn,Kp,Ki,del_t,Xerr_int)
    pJe = getPsuedo(curConfig,controls,del_t,limits)
    
if __name__ == '__main__':
    X,Xd,Xdn,Kp,Ki,del_t,Xerr_int_init = init(20,100)
    FeedbackControl(X,Xd,Xdn,Kp,Ki,del_t,Xerr_int_init)
    
