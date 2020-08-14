% Code to plot the workspace of the UR5 robot
% For MTRN4230 Group Project
% Author: Elliott Smith z3424674
% Date: 14/08/2020

clear all;
px = zeros([45*45*45*10*10,1]);
py = zeros([45*45*45*10*10,1]);
pz = zeros([45*45*45*10*10,1]);
i = 0;
% loops to input variables into forward kinematics
for theta1 = linspace(deg2rad(-180),deg2rad(180),45)
    for theta2 = linspace(deg2rad(-180),deg2rad(180),45)
        for theta3 = linspace(deg2rad(-180),deg2rad(180),45)
            for theta4 = linspace(deg2rad(-180),deg2rad(180),10)
                for theta5 = linspace(deg2rad(-180),deg2rad(180),10)        
                    i = i+1;
                    % assume the end effector is at position (0,0,0) in its
                    % frame
                    [px(i),py(i),pz(i)] = UR5_Forward_Kinematics(theta1,theta2,theta3,theta4,theta5,0,[0;0;0;1]);

                     
                end
            end
        end
    end
end
% plot the results
figure;
plot3(px,py,pz);
figure;
plot(px,py);
xlabel('X');
ylabel('Y');
figure;
plot(px,pz);
xlabel('X');
ylabel('Z');
figure;
plot(py,pz);
xlabel('Y');
ylabel('Z');
% these functions are copied from earlier work
function [y] = reduceToZero(x)
    if abs(x)<0.0001
        y = 0;
    else
        y = x;
    end
end

function [px,py,pz] = UR5_Forward_Kinematics(theta1,theta2,theta3,theta4,theta5,theta6,ee)
    T_01 = [reduceToZero(cos(theta1)) 0 reduceToZero(sin(theta1)) 0;
            reduceToZero(sin(theta1)) 0  -reduceToZero(cos(theta1)) 0;
            0 1 0 0.1625;
            0 0 0 1;
            ];

    T_12 = [reduceToZero(cos(theta2)) -reduceToZero(sin(theta2)) 0 reduceToZero(-0.425*cos(theta2));
            reduceToZero(sin(theta2)) reduceToZero(cos(theta2)) 0 reduceToZero(-0.425*sin(theta2));
            0 0 1 0;
            0 0 0 1;
            ];

    T_23 = [reduceToZero(cos(theta3)) -reduceToZero(sin(theta3)) 0 reduceToZero(-0.3922*cos(theta3));
            reduceToZero(sin(theta3)) reduceToZero(cos(theta3)) 0 reduceToZero(-0.3922*sin(theta3));
            0 0 1 0;
            0 0 0 1;
            ];

    T_34 = [reduceToZero(cos(theta4)) 0 reduceToZero(cos(theta4)) 0;
            reduceToZero(sin(theta4)) 0 -reduceToZero(cos(theta4)) 0;
            0 1 0 0.1333;
            0 0 0 1;
            ];

    T_45 = [reduceToZero(cos(theta5)) 0 -reduceToZero(sin(theta5)) 0;
            reduceToZero(sin(theta5)) 0 reduceToZero(cos(theta5)) 0;
            0 -1 0 0.0997;
            0 0 0 1;
            ];
        
    T_56 = [reduceToZero(cos(theta6)) -reduceToZero(sin(theta6)) 0 0;
            reduceToZero(sin(theta6)) reduceToZero(cos(theta6)) 0 0;
            0 0 1 0.0996;
            0 0 0 1;
            ];
        
    T = T_01 * T_12 * T_23 * T_34 * T_45 * T_56;
    p = T*ee;
    px = p(1,:);
    py = p(2,:);
    pz = p(3,:);
end
