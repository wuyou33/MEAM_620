function [pos_des, vel_des, acc_des, psi_des] = trajectory_generator(t, varargin)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
% t: A scalar, specifying inquiry time
% varargin: variable number of input arguments. In test_trajectory, this
% function will be called with the arguments trajectory_generator([], map, 
% path) so your code should be able to accomdate that. This can be done in
% part by saying map = varargin{1} and path = varargin{2}.
% Later it will be called with only t as an argument, so your code should
% also handle that.  See Matlab documentation for more information about
% varargin.
% map: 3D Occupancy grid map
% path: A N x 3 matrix where each row is (x, y, z) coordinate of a point in
% the path. N is the total number of points in the path..
% pos, etc.: Contains all information that pass into the controller, can be
% anything you like, but don't make it too large to make your code get slow

% Suggest using "persistent" vartiables to store map and path during the 
% initialization call of trajectory_generator, e.g.
% persistent map0 path0
% map0 = map;
% path0 = path;

% Written by Qiong Wang for MEAM 620 at the University of Pennsylvania.
% Feb.6th, 2013

%% PERSISTENT SETTING
persistent map0 path0 path_rdc cum_tm

%% REDUCE PATH IN INITIAL FILE
if(nargin > 2)
    %% INITIALIZE
    map0 = varargin{1};
    path0 = varargin{2};
    %% REDUCE PATH
    % MERGE points with same slope.
    diff_dat = diff(diff(path0));
    diff_mrk = [1; find(sum(diff_dat.^2,2)>= map0.res(1)^2/2) + 1; length(path0)];
    path_mrg = path0(diff_mrk,:);
    leng_mrg = length(path_mrg);

    % REDUCE the path by direct connection between merged path points.
    % Initialize persistent variable path_rdc to record reduced path.
    path_rdc = zeros(size(path_mrg,1),3);

    % Current start, start index number and end index number.
    st_crt = path_mrg(1,:);
    st_num = 1;
    path_rdc(st_num,:) = st_crt;
    ed_num = 2;

    % Reduce the path by connecting merged points, check each connection with
    % the current start and the points after it by collide().
    while ed_num < leng_mrg
        % Number of points based on the resolution.
        % pnts_num = ceil((path_mrg(ed_num,:)-st_crt)./map0.res);
        % Points between current start and end by interpolation.
        pnts_int = ndlinspace(st_crt, path_mrg(ed_num,:),100);
        % If not collide, keep on counting the index number of the end.
        if all(collide(map0, pnts_int) == 0)
            ed_num = ed_num + 1;
        % Else change the current start into the farthest non-collide point.
        else
            st_num = ed_num - 1;
            st_crt = path_mrg(st_num,:);
            path_rdc(st_num,:) = st_crt;
        end
        % If reaches the end of the merged points, record the end.
        if ed_num == leng_mrg
            if any(collide(map0, ndlinspace(st_crt, path_mrg(ed_num,:),100)))
                path_rdc(ed_num-1,:) = path_mrg(ed_num-1,:);
            end
            path_rdc(ed_num,:) = path_mrg(ed_num,:);
        end
    end

    % Remove the zero points in the reduced path.
    path_rdc = path_rdc(any(path_rdc~=0,2),:);

    %% TRAJECTORY PARAMETERS
    % Average velocity (m/s).
    v_avg = 1.2; %1.2; % 1.5
    % Length vector of reduced path (m).
    leng_rdc_vec = sqrt(sum((diff(path_rdc)).^2,2));
    % Trajectory time on every two points in the reduced path (s).
    traj_tm = [0; leng_rdc_vec .*(1-0.5*floor(leng_rdc_vec/20)).*(2-floor(leng_rdc_vec/20)).*(exp(-leng_rdc_vec/7)+floor(leng_rdc_vec/20))./ v_avg]; % traj_tm = [0; leng_rdc_vec ./ v_avg];
    % Cummulative time (s).
    cum_tm = cumsum(traj_tm);
    
else    
    %% QUINTIC POLYNOMIAL TRAJECTORIES
    % =============================== LOOP ================================
    for i = 1 : length(cum_tm)-1
         % The initial time and final time.
         q0 = path_rdc(i,:); 
         q1 = path_rdc(i+1,:);
         t0 = cum_tm(i); 
         tf = cum_tm(i+1);
         if t >= t0 && t < tf
             % The quintic ploynomial trajectory matrix.
             M = [ 1  t0 t0^2 t0^3   t0^4    t0^5;...
                   0  1  2*t0 3*t0^2 4*t0^3  5*t0^4;...
                   0  0  2    6*t0   12*t0^2 20*t0^3;...
                   1  tf tf^2 tf^3   tf^4    tf^5;...
                   0  1  2*tf 3*tf^2 4*tf^3  5*tf^4;...
                   0  0  2    6*tf   12*tf^2 20*tf^3];

             % Variable matrix.
             var = [q0; 0 0 0; 0 0 0; q1; 0 0 0; 0 0 0];

             % Coefficient matrix.
             a = M \ var;
             pos_des = a(1,:)' + a(2,:)'*t +a(3,:)'*t.^2 + a(4,:)'*t.^3 +a(5,:)'*t.^4 + a(6,:)'*t.^5;
             vel_des = a(2,:)' + 2*a(3,:)'*t +3*a(4,:)'*t.^2 +4*a(5,:)'*t.^3 +5*a(6,:)'*t.^4;
             acc_des = 2*a(3,:)' + 6*a(4,:)'*t +12*a(5,:)'*t.^2 +20*a(6,:)'*t.^3;
             
             % Yaw angle.
             psi_des = atan2(path_rdc(i+1,2) - path_rdc(i,2), path_rdc(i+1,1) - path0(i,1));
         end
    end
    
    % Quadrotor reaches the final, velocity and acceleration become zero.
    if t >= cum_tm(end)
        pos_des = path_rdc(end,:);
        vel_des = zeros(3,1);
        acc_des = zeros(3,1);
    end
end