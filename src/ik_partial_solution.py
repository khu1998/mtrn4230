import numpy as np
import math

def modifiedForwardKinematics(theta1,theta2,theta3,theta4,theta5,theta6):
    T_01 = np.array([[math.cos(theta1), 0, math.sin(theta1), 0],[math.sin(theta1), 0, -1*math.cos(theta1), 0], [0, 1, 0, 0.1625] , [0,0,0,1]], dtype='f')
    T_12 = np.array([[math.cos(theta2), -1*math.sin(theta2), 0, -0.425*math.cos(theta2)] , [math.sin(theta2), math.cos(theta2), 0, -0.425*math.sin(theta2)], [0,0,1,0], [0,0,0,1]], dtype='f')
    T_23 = np.array([[math.cos(theta3), -1*math.sin(theta3), 0, -0.3922*math.cos(theta3)], [math.sin(theta3), math.cos(theta3), 0, -0.3922*math.sin(theta3)], [0,0,1,0], [0,0,0,1]])
    T_34 = np.array([[math.cos(theta4), 0, math.sin(theta4), 0], [math.sin(theta4), 0, -1*math.cos(theta4), 0], [0,1,0,0.1333], [0,0,0,1]])
    T_45 = np.array([[math.cos(theta5), 0, -1*math.sin(theta5), 0], [math.sin(theta5), 0, math.cos(theta5), 0], [0, -1, 0, 0.0997], [0,0,0,1]])
    T_56 = np.array([[math.cos(theta6), -1*math.sin(theta6), 0, 0], [math.sin(theta6), math.cos(theta6), 0, 0], [0, 0, 1, 0.0996] ,[0,0,0,1]])
    T_02 = np.matmul(T_01,T_12)
    T_03 = np.matmul(T_02,T_23)
    T_04 = np.matmul(T_03,T_34)
    T_05 = np.matmul(T_04,T_45)
    T_06 = np.matmul(T_05,T_56)
    return T_06

def inverseKinematics(T_06):
    
    d1 = 0.1625
    d4 = 0.1333
    d5 = 0.0997
    d6 = 0.0996
    a2 = -0.425
    a3 = -0.3922

    # joint positions (angular position)
    # index i = joint angle i i.e. theta[1] = theta_1
    theta = [0,0,0,0,0,0,0]

    P_06 = np.matmul(T_06,np.array([[0],[0],[0],[1]]))
    P_05 = np.matmul(T_06,np.array([[0],[0],[-d6],[1]]))

    # --------- theta1 ---------  
    theta[1] = math.atan2(P_05[1],P_05[0])+math.acos(d4/math.sqrt(P_05[0]*P_05[0]+P_05[1]*P_05[1]))+math.pi/2
    # theta1 = math.atan2(P_05[1],P_05[0])-math.acos(d4/math.sqrt(P_05[0]*P_05[0]+P_05[1]*P_05[1]))+math.pi/2

    # --------- theta5 --------- 
    num5 = P_06[0]*math.sin(theta1)-P_06[1]*math.cos(theta[1])-d4
    theta[5] = math.acos(num5/d6)
    # theta5 = -math.acos(num5/d6)

    # --------- theta6 --------- 
    T_60 = np.linalg.inv(T_06)

    X_60 = np.array([[T_60[0,0]], [T_60[1,0]], [T_60[2,0]]])
    Y_60 = np.array([[T_60[0,1]], [T_60[1,1]], [T_60[2,1]]])

    arg1 = (-X_60[1]*math.sin(theta[1])+Y_60[1]*math.cos(theta[1]))/math.sin(theta[5])
    arg2 = (X_60[0]*math.sin(theta[1])-Y_60[0]*math.cos(theta[1]))/math.sin(theta[5])
    theta[6] = math.atan2(arg1, arg2)

    # --------- theta3 --------- 
    
    T_01 = np.array([[math.cos(theta[1]), 0, math.sin(theta[1]), 0],[math.sin(theta[1]), 0, -1*math.cos(theta[1]), 0], [0, 1, 0, 0.1625] , [0,0,0,1]], dtype='f')
    T_45 = np.array([[math.cos(theta[5]), 0, -1*math.sin(theta[5]), 0], [math.sin(theta[5]), 0, math.cos(theta[5]), 0], [0, -1, 0, 0.0997], [0,0,0,1]])
    T_56 = np.array([[math.cos(theta[6]), -1*math.sin(theta[6]), 0, 0], [math.sin(theta[6]), math.cos(theta[6]), 0, 0], [0, 0, 1, 0.0996] ,[0,0,0,1]])

    T_64 = np.linalg.inv(np.matmul(T_45,T_56))
    T_10 = np.linalg.inv(T_01)
    T_14 = np.matmul(np.matmul(T_10,T_06),T_64)

    P_14 = np.matmul(T_14,np.array([[0],[0],[0],[1]]))
    P_14xz_len = math.sqrt(P_14[0]*P_14[0]+P_14[1]*P_14[1])
    theta[3] = math.acos((P_14xz_len*P_14xz_len-a2*a2-a3*a3)/2/a2/a3)
    # theta[3] = -math.acos((P_14xz_len*P_14xz_len-a2*a2-a3*a3)/2/a2/a3)

    # --------- theta2 --------- 
    phi1 = math.atan2(-P_14[1],-P_14[0])
    phi2 = math.asin(-a3*math.sin(theta[3])/P_14xz_len)
    theta[2] = phi1 - phi2

    # --------- theta4 --------- 
    T_01 = np.array([[math.cos(theta[1]), 0, math.sin(theta[1]), 0],[math.sin(theta[1]), 0, -1*math.cos(theta[1]), 0], [0, 1, 0, 0.1625] , [0,0,0,1]], dtype='f')
    T_12 = np.array([[math.cos(theta[2]), -1*math.sin(theta[2]), 0, -0.425*math.cos(theta[2])] , [math.sin(theta[2]), math.cos(theta[2]), 0, -0.425*math.sin(theta[2])], [0,0,1,0], [0,0,0,1]], dtype='f')
    T_23 = np.array([[math.cos(theta[3]), -1*math.sin(theta[3]), 0, -0.3922*math.cos(theta[3])], [math.sin(theta[3]), math.cos(theta[3]), 0, -0.3922*math.sin(theta3)], [0,0,1,0], [0,0,0,1]])

    T_02 = np.matmul(T_01,T_12)
    T_03 = np.matmul(T_02,T_23)

    T_30 = np.linalg.inv(T_03)
    T_54 = np.linalg.inv(T_45)

    T_40 = np.matmul(np.matmul(T_45,T_56),T_60)

    T_04 = np.linalg.inv(T_40)
    T_30 = np.linalg.inv(T_03)
    T_34 = np.matmul(T_30,T_04)

    X_34 = np.array([[T_34[0,0]], [T_34[1,0]], [T_34[2,0]]])
    theta[4] = math.atan2(X_34[1],X_34[0])

    print("inverse:", theta[1], theta[2], theta[3], theta[4], theta[5],theta[6])

    return theta

if __name__=="__main__":
    
    theta1 = math.pi/2.3
    theta2 = math.pi/4.2
    theta3 = math.pi/3.4
    theta4 = math.pi/8.123
    theta5 = math.pi/4.2
    theta6 = math.pi/10.5

    print ("input:", [theta1,theta2,theta3,theta4,theta5,theta6])
    T_06 = modifiedForwardKinematics(theta1,theta2,theta3,theta4,theta5,theta6)
    
    _theta = inverseKinematics(T_06)  
    theta = [theta1,theta2,theta3,theta4,theta5,theta6]
    zip_theta = zip(theta, _theta[1:6])
    error = []
    for forward_theta, inverse_theta in zip_theta:
        error.append((forward_theta-inverse_theta)/forward_theta*100)
    print("error (%):",error)
