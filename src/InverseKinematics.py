import numpy as np
import math

class InverseKinematics()
    @staticmethod
    def getJointPositions(ee):
        alpha = np.array([0,math.pi/2,0,0,math.pi/2,-math.pi/2,0])
        a = np.array([0,0,0.425,0.39225,0,0,0])
        d = np.array([0,0.08916,0,0,0.10915,0.09456,0.0823])
        jp = [0,0,0,0,0,0]

        jp[0] = alpha[1]+alpha[2]+math.pi/2

        jp[4] = 
        jp[5] = 0      

        return jp

    def getJointPositionsFromConveyorPose(ee)
        
        pass