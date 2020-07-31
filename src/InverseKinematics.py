import numpy as np
import math

class InverseKinematics()
    @staticmethod
    def getJointPositions(ee):
        
        x = ee[0]
        y = ee[1]
        z = ee[2]

        # DH parameters
        d1 = 0.1625
        d4 = 0.1333
        d5 = 0.0997
        d6 = 0.0996
        a2 = -0.425
        a3 = -0.3922
        
        # rotation angles of the end effector in global frame.
        phi_x = math.pi
        phi_y = math.pi
        phi_z = math.pi
        Rx = np.array([[1, 0, 0], [0, math.cos(phi_x), -math.sin(phi_x)], [0, math.sin(phi_x), math.cos(phi_x)]],dtype='f')
        Ry = np.array([[math.cos(phi_y), 0, -math.sin(phi_y)], [0, 1, 0], [math.sin(phi_y), 0, math.cos(phi_y)]],dtype='f')
        Rz = np.array([[math.cos(phi_z), -math.sin(phi_z), 0], [math.sin(phi_z), math.cos(phi_z), 0], [0, 0, 1]],dtype='f')

        # define resultant multiplication matrix by multiplying (or simply use) rotation matrices
        R = Rz

        # create transformation matrix for inverse kinematics
        T_06 = np.array([[R[0,0],R[0,1],R[0,2],x],[R[1,0],R[1,1],R[1,2],y],[R[2,0],R[2,1],R[2,2],z],[0,0,0,1]],dtype='f')

        # joint positions (angular positions)
        # index i = joint angle i i.e. theta[1] = theta_1
        theta = [0,0,0,0,0,0,0]

        # --------- theta1 ---------  
        P_06 = np.matmul(T_06,np.array([[0],[0],[0],[1]]))
        P_05 = np.matmul(T_06,np.array([[0],[0],[-d6],[1]]))

        theta[1] = math.atan2(P_05[1],P_05[0])+math.acos(d4/math.sqrt(P_05[0]*P_05[0]+P_05[1]*P_05[1]))+math.pi/2
        # alternate position
        # theta[1] = math.atan2(P_05[1],P_05[0])-math.acos(d4/math.sqrt(P_05[0]*P_05[0]+P_05[1]*P_05[1]))+math.pi/2

        # --------- theta5 --------- 
        num5 = P_06[0]*math.sin(theta[1])-P_06[1]*math.cos(theta[1])-d4
        theta[5] = math.acos(num5/d6)
        # alternate position
        # theta5 = -math.acos(num5/d6)

        # --------- theta6 --------- 
        T_60 = np.linalg.inv(T_06)
        # rotation axis of the corrdinate frames
        X_60 = np.array([[T_60[0,0]], [T_60[1,0]], [T_60[2,0]]])
        Y_60 = np.array([[T_60[0,1]], [T_60[1,1]], [T_60[2,1]]])

        arg1 = (-X_60[1]*math.sin(theta[1])+Y_60[1]*math.cos(theta[1]))/math.sin(theta[5])
        arg2 = (X_60[0]*math.sin(theta[1])-Y_60[0]*math.cos(theta[1]))/math.sin(theta[5])
        theta[6] = math.atan2(arg1, arg2)

        # --------- theta3 --------- 
        # using derivation from forward kinematics and theta found previously, construct
        # necessary T matrices to apply composition rule.
        T_01 = np.array([[math.cos(theta[1]), 0, math.sin(theta[1]), 0],[math.sin(theta[1]), 0, -1*math.cos(theta[1]), 0], [0, 1, 0, 0.1625] , [0,0,0,1]], dtype='f')
        T_45 = np.array([[math.cos(theta[5]), 0, -1*math.sin(theta[5]), 0], [math.sin(theta[5]), 0, math.cos(theta[5]), 0], [0, -1, 0, 0.0997], [0,0,0,1]])
        T_56 = np.array([[math.cos(theta[6]), -1*math.sin(theta[6]), 0, 0], [math.sin(theta[6]), math.cos(theta[6]), 0, 0], [0, 0, 1, 0.0996] ,[0,0,0,1]])

        # composition rule to derive 1T4
        T_64 = np.linalg.inv(np.matmul(T_45,T_56))
        T_10 = np.linalg.inv(T_01)
        T_14 = np.matmul(np.matmul(T_10,T_06),T_64)

        # translation component of 1T4
        P_14 = np.matmul(T_14,np.array([[0],[0],[0],[1]]))
        P_14xz_len = math.sqrt(P_14[0]*P_14[0]+P_14[1]*P_14[1])
        theta[3] = math.acos((P_14xz_len*P_14xz_len-a2*a2-a3*a3)/2/a2/a3)
        # alternate solution
        # theta[3] = -math.acos((P_14xz_len*P_14xz_len-a2*a2-a3*a3)/2/a2/a3)

        # --------- theta2 --------- 
        phi1 = math.atan2(-P_14[1],-P_14[0])
        phi2 = math.asin(-a3*math.sin(theta[3])/P_14xz_len)
        theta[2] = phi1 - phi2

        # --------- theta4 --------- 
        # using derivation from forward kinematics and theta found previously, construct
        # necessary T matrices to apply composition rule.
        T_01 = np.array([[math.cos(theta[1]), 0, math.sin(theta[1]), 0],[math.sin(theta[1]), 0, -1*math.cos(theta[1]), 0], [0, 1, 0, 0.1625] , [0,0,0,1]], dtype='f')
        T_12 = np.array([[math.cos(theta[2]), -1*math.sin(theta[2]), 0, -0.425*math.cos(theta[2])] , [math.sin(theta[2]), math.cos(theta[2]), 0, -0.425*math.sin(theta[2])], [0,0,1,0], [0,0,0,1]], dtype='f')
        T_23 = np.array([[math.cos(theta[3]), -1*math.sin(theta[3]), 0, -0.3922*math.cos(theta[3])], [math.sin(theta[3]), math.cos(theta[3]), 0, -0.3922*math.sin(theta[3])], [0,0,1,0], [0,0,0,1]])

        # composition rule
        T_02 = np.matmul(T_01,T_12)
        T_03 = np.matmul(T_02,T_23)

        #inverse matrix to get correct ordering for composition.
        T_30 = np.linalg.inv(T_03)
        T_54 = np.linalg.inv(T_45)

        # composition rule
        T_40 = np.matmul(np.matmul(T_45,T_56),T_60)

        T_04 = np.linalg.inv(T_40)
        T_30 = np.linalg.inv(T_03)
        # finally obtain 3T4
        T_34 = np.matmul(T_30,T_04)

        X_34 = np.array([[T_34[0,0]], [T_34[1,0]], [T_34[2,0]]])
        theta[4] = math.atan2(X_34[1],X_34[0])

        # print("inverse:", theta[1], theta[2], theta[3], theta[4], theta[5],theta[6])
        return theta

    def getJointPositionsFromConveyorPose(ee)
        pass