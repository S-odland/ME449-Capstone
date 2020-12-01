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

def writeCSV(line):
    data = pd.DataFrame(line)
    data.to_csv("nextstate.csv",header=False,index=False)

def genRef(curConfig,controls,del_t,limits):
    refConfig = NS.simControls(curConfig,controls,del_t,limits)
    return refConfig

def main():
    pass


if __name__ == '__main__':
    main()