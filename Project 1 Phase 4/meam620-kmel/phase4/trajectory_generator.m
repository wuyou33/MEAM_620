function [pos, phi, phi_d, vel, acc, total_t] = trajectory_generator(t, varargin)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
% t: A scalar, specifying inquiry time
%
% varargin: variable number of input arguments. In test_trajectory, this
% function will be called with the arguments trajectory_generator([], map, path) 
% so your code should be able to accomdate that. This can be done in part by 
% saying map = varargin{1} and path = varargin{2}.
% Later it will be called with only t as an argument, so your code should
% also handle that.  See Matlab documentation for more information about
% varargin.
%
% map: 3D Occupancy grid map
% path: A N x 3 matrix where each row is (x, y, z) coordinate of a point in the path. N is the total number of points in the path
%
% pos, etc: Contains all information that pass into the controller, can be
% anything you like, but do not make it too large since your code will get
% slow. As in phase 2 you should have the desired position output as the
% first arugment and anything else you want later (etc). 

% Suggest using "persistent" vartiables to store map and path during the initialization call of trajectory_generator, e.g.
% persistent map0 path0f
% map0 = map;
% path0 = path;

persistent map0 path0 path_vel path_size time k_traj end_time valid_start;
path_vel_max = 20;  % maximum of speed, 20m/s.

if nargin > 2
    map0 = varargin{1};
    path0 = varargin{2};
    path_size = size(path0, 1);
    end_time = 0;
    valid_start = 1;
%     xy_res = map0{2}(1);
%     z_res = map0{2}(2);
    
    % calculate the path_velocity for each path
    path_vel = zeros(path_size - 1, 3);
    vect_t = zeros(path_size - 1, 3);
    
    vect = path0(2:end, :) - path0(1:end-1, :);
    path_length = zeros(path_size - 1, 1);  % calculate the length of each segment of path
    for i = 1: path_size - 1
        path_length(i) = norm(vect(i, :));
        vect_t(i, :) = vect(i, :) / path_length(i);
    end
    
    for i = 1: path_size - 1
        path_vel(i, :) = vect_t(i, :) * path_vel_max;
%         path_vel(i, 3) = path_vel(i, 3) / z_res;
    end
    
    
    % set the time for each point
    time = zeros(path_size, 1);
    temp = 0;
    for i = 2: path_size
        temp = path_length(i - 1) / norm(path_vel(i - 1, :)) + temp;
        time(i) = temp;
    end
    
    
    return;
end

total_t = time(end);

if t >= total_t
    pos = path0(end, :);
    phi = 0;
    phi_d = 0;
    vel = [0 0 0];
    acc = [0 0 0];
    return;
end


if t >= end_time
    % find the longest straight line without collide
    for i  = path_size:-1:valid_start
        start_point = path0(valid_start, :);
        end_point = path0(i, :);
        len = end_point - start_point;
        num = ceil(norm(len) / 0.01) + 1;
        
        points = zeros(num, 3);
        points(:, 1) = linspace(start_point(1), end_point(1), num);
        points(:, 2) = linspace(start_point(2), end_point(2), num);
        points(:, 3) = linspace(start_point(3), end_point(3), num);
        
        if ~sum(collide(map0, points))    % comment out for phase4
            % trajectory from start_point to end_point
            start_time = time(valid_start);
            end_time = time(i);
            valid_end = i;
            
            A = [start_time^5, start_time^4, start_time^3, start_time^2, start_time, 1;...
                5*start_time^4, 4*start_time^3, 3*start_time^2, 2*start_time, 1, 0;...
                20*start_time^3, 12*start_time^2, 6*start_time, 2, 0, 0;...
                end_time^5, end_time^4, end_time^3, end_time^2, end_time, 1;...
                5*end_time^4, 4*end_time^3, 3*end_time^2, 2*end_time, 1, 0;...
                20*end_time^3, 12*end_time^2, 6*end_time, 2, 0, 0];

            b = [start_point; 0, 0, 0; 0, 0, 0; end_point; 0, 0, 0; 0, 0, 0];
            k_traj = A\b;
            
            acc_max_t = [start_time + (end_time - start_time)/4, start_time + (end_time - start_time)/4*3];
            acc_max = [20*acc_max_t(1)^3, 12*acc_max_t(1)^2, 6*acc_max_t(1), 2, 0, 0;...
                20*acc_max_t(2)^3, 12*acc_max_t(2)^2, 6*acc_max_t(2), 2, 0, 0] * k_traj;
            
            % use longer time for the trajectory if the accelerations are
            % larger than these values
            while norm(acc_max(1, 1:2)) >= 5.5 || abs(acc_max(1, 3)) >= 14 || norm(acc_max(2, 1:2)) >= 5.5 || abs(acc_max(2, 3)) >= 4
                time(valid_start + 1:end) = time(valid_start + 1:end) + 0.1;
                
                end_time = time(i);
                
                A = [start_time^5, start_time^4, start_time^3, start_time^2, start_time, 1;...
                    5*start_time^4, 4*start_time^3, 3*start_time^2, 2*start_time, 1, 0;...
                    20*start_time^3, 12*start_time^2, 6*start_time, 2, 0, 0;...
                    end_time^5, end_time^4, end_time^3, end_time^2, end_time, 1;...
                    5*end_time^4, 4*end_time^3, 3*end_time^2, 2*end_time, 1, 0;...
                    20*end_time^3, 12*end_time^2, 6*end_time, 2, 0, 0];

                b = [start_point; 0, 0, 0; 0, 0, 0; end_point; 0, 0, 0; 0, 0, 0];
                k_traj = A\b;

                acc_max_t = [start_time + (end_time - start_time)/4, start_time + (end_time - start_time)/4*3];
                acc_max = [20*acc_max_t(1)^3, 12*acc_max_t(1)^2, 6*acc_max_t(1), 2, 0, 0;...
                20*acc_max_t(2)^3, 12*acc_max_t(2)^2, 6*acc_max_t(2), 2, 0, 0] * k_traj;
            end
            
            valid_start = valid_end;
            break;
        end
    end
end

pos = [t^5, t^4, t^3, t^2, t, 1] * k_traj;
vel = [5*t^4, 4*t^3, 3*t^2, 2*t, 1, 0] * k_traj;
acc = [20*t^3, 12*t^2, 6*t, 2, 0, 0] * k_traj;
phi = 0;
phi_d = 0;

time(end)

end