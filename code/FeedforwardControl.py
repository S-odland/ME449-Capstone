## Library Imports

import modern_robotics as mr 
import numpy as np
import math as m 
import pandas as pd
import itertools
import importlib
import TrajectoryGenerator as TG
import NextState as NS

## INPUTS ##
## 1. The current actual end-effector configuration X (Tse)
## 2. The current end-effector reference configuration Xd (Tse,d)
## 3. The end-effector reference configuration at the next timestep in the reference
##    trajectory, Xd,next(Tse,d,next) at a timestep delta_t later
## 4. The PI gain matrices Kp and Ki
## 5. The timestep delta_t between reference trajectory configurations