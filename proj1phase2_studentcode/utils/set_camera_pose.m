function set_camera_pose(h, position, orientation)
    gca = get(h, 'CurrentAxes');
    campos(gca, position);
    Ryp = ypr_to_R(orientation(1), orientation(2), 0);
    Rr  = ypr_to_R(0, 0, orientation(3));    
    t = Ryp * [1; 0; 0] + position;
    r = Rr  * [0; 0; 1];
    camtarget(gca, t);
    camup(gca, r);
end