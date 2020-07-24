import numpy as np
import math
class ForwardKinematics():
    @staticmethod
    def getGlobalPositionOfEndEffector(theta1,theta2,theta3,theta4,theta5,theta6,ee):
        T_01 = np.array([[math.cos(theta1), 0, math.sin(theta1), 0],[math.sin(theta1), 0, -1*math.cos(theta1), 0], [0, 1, 0, 0.1625] , [0,0,0,1]], dtype='f')
        T_12 = np.array([[math.cos(theta2), -1*math.sin(theta2), 0, -0.425*math.cos(theta2)] , [math.sin(theta2), math.cos(theta2), 0, -0.425*math.sin(theta2)], [0,0,1,0], [0,0,0,1]], dtype='f')
        T_23 = np.array([[math.cos(theta3), -1*math.sin(theta3), 0, -0.3922*math.cos(theta3)], [math.sin(theta4), math.cos(theta3), 0, -0.3922*math.sin(theta3)], [0,0,1,0], [0,0,0,1]])
        T_34 = np.array([[math.cos(theta4), 0, math.sin(theta4), 0], [math.sin(theta4), 0, -1*math.cos(theta4), 0], [0,1,0,0.1333], [0,0,0,1]])
        T_45 = np.array([[math.cos(theta5), 0, -1*math.sin(theta5), 0], [math.sin(theta5), 0, math.cos(theta5), 0], [0, -1, 0, 0.0997], [0,0,0,1]])
        T_56 = np.array([[math.cos(theta6), -1*math.sin(theta6), 0, 0], [math.sin(theta6), math.cos(theta6), 0, 0], [0, 0, 1, 0.0996] ,[0,0,0,1]])
        T_02 = np.matmul(T_01,T_12)
        T_03 = np.matmul(T_02,T_23)
        T_04 = np.matmul(T_03,T_34)
        T_05 = np.matmul(T_04,T_45)
        T_06 = np.matmul(T_05,T_56)
        return np.matmul(T_06,ee) 

