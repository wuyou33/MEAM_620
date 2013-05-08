function [pos, vel, acc, euler, omegala] = trajectory_generator(t, varargin)
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
persistent map0 path0 RealPath  velvect accvect
global traj
    max_vel=[2 2 2];
    dt = 0.1;
    tacc=5;

if nargin > 1
    map0 = varargin{1};
    path0 = varargin{2};
    RealPath =[];
    map = map0;
    path = path0;
%     count = 1;
%     EndFlag=0;
% 
%     Startidx = 1; % piecewise path starting index. = the column number
%     Endidx = size(path,1); %end for piece
% 
    % set up new trajectory with piecewise
%     while EndFlag ~= 1
% 
%         collidetest=[linspace(path(Startidx,1),path(Endidx,1));...
%                     linspace(path(Startidx,2),path(Endidx,2));...
%                     linspace(path(Startidx,3),path(Endidx,3))]';
%         %if sum(collide(map,collidetest)>0)
%         if(1)
%             Endidx = Endidx-1;
%         else
%             RealPath(count,:)=path(Endidx,:);
%             count = count+1;
%             if Endidx==size(path,1)
%                 EndFlag=1;
%                 RealPath=[path(1,:);RealPath];
% 
%             end
%             Startidx = Endidx;
%             Endidx = size(path,1);
% 
%         end
%     end
    % add time segment lengths to each row of RealPath
   %   RealPath=[RealPath, [ 0; (sqrt(sum(diff(RealPath).^2,2))./max_vel(1)) ]]; 

    RealPath=path0;

%     Robo kit. The more fine the time step is the slower and better runs
    traj=mstraj(RealPath(2:end,1:3), max_vel, [], RealPath(1,1:3), dt, tacc);
    while ( sum(collide(map,traj))>0 && tacc~=0 )
        traj=mstraj(RealPath(2:end,1:3), max_vel, [], RealPath(1,1:3), dt, tacc);
        tacc=floor(tacc*3/4);
    end
    
    velvect = [diff(traj); [0 0 0]]./dt;
    accvect = [diff(velvect); [0 0 0]]./dt;
    
end

if ~isempty(t)
    x_dd=0;
    y_dd=0;
    z_dd=0;
    space=(linspace(0,sum(RealPath(:,4)),size(traj,1))-t);
    idx=find(space>0) ;
    if ~isempty(idx)
        idx=idx(1);

        pos=traj(idx,:)';
        vel=velvect(idx,:)';
        acc=accvect(idx,:)';
    %     euler=[0;0;0];
    %     omegala=[0;0;0];

    %     pos=[0;-4.8;5];
    %     vel=[0;0;0];
    %     acc=[0;0;0];
        euler=[0;0;0];
        omegala=[0;0;0];
    else 
        pos=traj(end,:)';
        vel=[0;0;0];
        acc=[0;0;0];
        euler=[0;0;0];
        omegala=[0;0;0];

    end
    % the brakes! :P
    % if t>=.96*sum(RealPath(:,4))
        
    % end

else
    pos=traj(1,:)';
    vel=[0;0;0];
    acc=[0;0;0];
    euler=[0;0;0];
    omegala=[0;0;0];

    
end




    

