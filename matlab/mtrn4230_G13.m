clear all;
close all;


ipaddress = '10.0.0.75';
robotType = 'Gazebo'
rosshutdown;
rosinit(ipaddress);
blockposes = rossubscriber('/gazebo/link_states');
pause(2);
posdata = receive(blockposes,10);
imSub = rossubscriber('/camera/color/image_raw');
pcSub = rossubscriber('/camera/depth/image_raw');
infoSub = rossubscriber('/camera/color/camera_info'); 
posPub = rospublisher('/cv_pos','std_msgs/String');
posSub = rossubscriber('/cv_pos');

rostopic list;

% id, type, x, y, z
positions = [{0, 'red', 0, 0, 0.6},
                {1, 'blue', 0, 0.1, 0.6},
                {2, 'yellow', 0.1, 0, 0.6}]

% pixel positions are in x,y format
pixel_positions = [167 87; 200 200; 400 400;];
            
i = 0;
while true
    figure(1)
    testIm  = readImage(imSub.LatestMessage);
    imshow(testIm);
    
    figure(2);
    depthIm  = readImage(pcSub.LatestMessage);
    depth_max = max(max(depthIm))
    depth_min = min(min(depthIm))
    depthImDisplay = (depthIm-depth_min)/(depth_max - depth_min);
    % for grayscale image, matlab expects values from 0.0 to 1.0, not 0 to
    % 255
    imshow(depthImDisplay);
    
    for i = 1:length(pixel_positions)
        x = pixel_positions(i,1);
        y = pixel_positions(i,2);
        p3d = toCamera(x, y, depthIm(y,x)) % image reference in row, col format (ie y, x)
        gp3d = toGlobal(p3d)
    end
    
    cv_msg = rosmessage(posPub);
    cv_msg.Data = sprintf("%d|%s",i,toString(positions))
    send(posPub,cv_msg);
    
    i = i + 1;
    pause(1)
%     msg = receive(posSub,3)
    
      
end

function P3D = toCamera(px, py, d)
% from http://nicolas.burrus.name/index.php/Research/KinectCalibration
    % variables themselves come from the infoSub (camera_info subscriber)
    % messages
    fx_d = 554.255971187975;
    fy_d = 554.255971187975;
    cx_d = 320.5;
    cy_d = 240.5;
    P3D.x = (px - cx_d) * d / fx_d;
    P3D.y = (py - cy_d) * d / fy_d;
    P3D.z = d;
end

function P3D = toGlobal(cam_p3d)
    P3D.x = cam_p3d.y;
    P3D.y = -cam_p3d.x;
    P3D.z = 2.0 - cam_p3d.z;
end

function msgString = toString(positions)
    msgString=""
    shape = size(positions);
    num_entries = shape(1);
    for i = 1:num_entries
        p = positions(i,:);
        msgString = msgString + sprintf('%d,%s,%0.5f,%0.5f,%0.5f|',p{1},p{2},p{3},p{4},p{5});
    end
end
