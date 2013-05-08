function [ des_pos ] = plot_des_traj( total_t, path )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    t = 0:0.1:total_t;
    trajectory_generator_no_obstacle([], [], path)
    des_pos = zeros(3, size(t, 2));
    for i = 1:size(t, 2);
        des_pos(:, i) = trajectory_generator_no_obstacle(t(i))';
    end
end
