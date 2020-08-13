function P3D = toGlobal(cam_p3d)
    P3D.x = -cam_p3d.x;
    P3D.y = cam_p3d.y;
    P3D.z = 2.0 - cam_p3d.z;
end