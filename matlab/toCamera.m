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