clear all;
close all;


ipaddress = '10.0.0.77';
setenv('ROS_MASTER_URI',"http://"+ipaddress+":11311")

robotType = 'Gazebo'
rosshutdown;
rosinit(ipaddress);
blockposes = rossubscriber('/gazebo/link_states');
pause(2);
posdata = receive(blockposes,10);
statusSub = rossubscriber('/cv_status',@exampleHelperROSChatterCallback);
status2Sub = rossubscriber('cv_status',@exampleHelperROSChatterCallback);

while true
    pause(1);
end

function myCallback(src, msg)
    disp('A service client is calling');
end