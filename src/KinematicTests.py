import numpy as np
import math
from ForwardKinematics import ForwardKinematics

ee = np.array([0.0,0.0,0.0,1.0])

basePositionOfEE = ForwardKinematics.getGlobalPositionOfEndEffector(0,0,0,0,0,0,ee)
print(basePositionOfEE)
x = basePositionOfEE[0]
y = basePositionOfEE[1]
z = basePositionOfEE[2]
print(x,y,z)