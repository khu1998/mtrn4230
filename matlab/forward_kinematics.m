syms theta1 theta2 theta3 theta4 theta5 theta6

T_01 = [cos(theta1) 0 sin(theta1) 0;
        sin(theta1) 0  -cos(theta1) 0;
        0 1 0 0.1625;
        0 0 0 1;
        ];

T_12 = [cos(theta2) -sin(theta2) 0 -0.425*cos(theta2);
        sin(theta2) cos(theta2) 0 -0.425*sin(theta2);
        0 0 1 0;
        0 0 0 1;
        ];

T_23 = [cos(theta3) -sin(theta3) 0 -0.3922*cos(theta3);
        sin(theta3) cos(theta3) 0 -0.3922*sin(theta3);
        0 0 1 0;
        0 0 0 1;
        ];

T_34 = [cos(theta4) 0 cos(theta4) 0;
        sin(theta4) 0 -cos(theta4) 0;
        0 1 0 0.1333;
        0 0 0 1;
        ];

T_45 = [cos(theta5) 0 -sin(theta5) 0;
        sin(theta5) 0 cos(theta5) 0;
        0 -1 0 0.0997;
        0 0 0 1;
        ];

T_56 = [cos(theta6) -sin(theta6) 0 0;
        sin(theta6) cos(theta6) 0 0;
        0 0 1 0.0996;
        0 0 0 1;
        ];

T = T_01 * T_12 * T_23 * T_34 * T_45 * T_56;
disp(T(1:3,4));
