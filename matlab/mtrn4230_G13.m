clear all;
close all;


ipaddress = '10.0.0.73';
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
    
    cv_msg = rosmessage(posPub);
    cv_msg.Data = sprintf("%d|%s",i,toString(positions))
    send(posPub,cv_msg);
    
    i = i + 1;
    pause(1)
%     msg = receive(posSub,3)
    
      
end

function msgString = toString(positions)
    msgString=""
    shape = size(positions);
    num_entries = shape(1);
    for i = 1:num_entries
        p = positions(i,:);
        msgString = msgString + sprintf('%d,%s,%0.5f,%0.5f,%0.5f|',p{1},p{2},p{3},p{4},p{5})
    end
end
