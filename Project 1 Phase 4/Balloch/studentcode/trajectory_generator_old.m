function s_des = trajectory_generator(t, s, map, path)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
% t: A scalar, specifying inquiry time
% s: State of the robot
% map: 3D Occupancy grid map
% path: A N x 3 matrix where each row is (x, y, z) coordinate of a point in the path. N is the total number of points in the path
% s_des: Contains all information that pass into the controller, can be anything you like, but do not make it too large since your code will get slow

% Suggest using "persistent" variables to store map and path during the initialization call of trajectory_generator, e.g.
% persistent map0 path0
% map0 = map;
% path0 = path;

% use hermite? is this overkill? (pp) ...pchip
% try spline with interp2
% 
% also maybe nonlinear least squares:
% f = fittype('a*sin(b*x)'); 
% fit1 = fit(xdata,ydata,f,'StartPoint',[1 1]);
% Identify "outliers" as points at a distance greater than 1.5 standard deviations from the baseline model, and refit the data with the outliers excluded:
% fdata = feval(fit1,xdata); 
% I = abs(fdata - ydata) > 1.5*std(ydata); 
% outliers = excludedata(xdata,ydata,'indices',I);
% 
% fit2 = fit(xdata,ydata,f,'StartPoint',[1 1],'Exclude',outliers);
global MapData

persistent map0 path0 RealPath

max_vel=1;

if nargin < 4
    map = map0;
    path = path0;
else
    RealPath =[];
    map0 = map;
    path0 = path;
    count = 1;
    EndFlag=0;

    Startidx = 1; % piecewise path starting index. = the column number
    Endidx = size(path,1); %end for piece

    % set up new trajectory with splines
    while EndFlag ~= 1

        collidetest=[linspace(path(Startidx,1),path(Endidx,1));...
                    linspace(path(Startidx,2),path(Endidx,2));...
                    linspace(path(Startidx,3),path(Endidx,3))]';
        if sum(collide(map,collidetest)>0)
            Endidx = Endidx-1;
        else
            RealPath(count,:)=path(Endidx,:);
            count = count+1;
            if Endidx==size(path,1)
                EndFlag=1;
                RealPath=[path(1,:);RealPath];

            end
            Startidx = Endidx;
            Endidx = size(path,1);

        end
    end
    % add time segment lengths to each row of RealPath
    RealPath=[RealPath, [ 0; (sqrt(sum(diff(RealPath).^2,2))./max_vel) ]];
end



% xx=min(path(:,1)):0.01:max(path(:,1));
% yy=min(path(:,2)):0.01:max(path(:,2));
% Y=pchip(RealPath(:,1),RealPath(:,2),xx);
% Z=pchip(RealPath(:,1),RealPath(:,3),xx);


s_des.pos=[0;0;0];
s_des.vel=[0;0;0];
s_des.acc=[0;0;0];
s_des.euler=[0;0;0];
s_des.omegala=[0;0;0];


% find desired state if time given
if ~isempty(t)
    fflag=0;
    x_dd=0;
    y_dd=0;
    z_dd=0;

    for i=2:size(RealPath,1)

        if (t<RealPath(i,4) && fflag==0)
            t_perc=(t-sum(RealPath(1:(i-1),4)))/RealPath(i,4);
            x = t_perc*(RealPath(i,1)-RealPath(i-1,1))+RealPath(i-1,1);
            x_d = (RealPath(i,1)-RealPath(i-1,1))/RealPath(i,4);
            y = t_perc*(RealPath(i,2)-RealPath(i-1,2))+RealPath(i-1,2);
            y_d = (RealPath(i,2)-RealPath(i-1,2))/RealPath(i,4);
            z = t_perc*(RealPath(i,3)-RealPath(i-1,3))+RealPath(i-1,3);
            z_d = (RealPath(i,3)-RealPath(i-1,3))/RealPath(i,4);

    %         if (t>spr*.95)
    %             x=1;
    %             y=0;
    %             z=0;
    %             x_d=0;
    %             y_d=0;
    %             z_d=0;
    %             x_dd=0;
    %             y_dd=0;
    %             z_dd=0;
    %         end
            fflag=1;
        elseif t>sum(RealPath(:,4))
            x=RealPath(end,1);
            y=RealPath(end,2);
            z=RealPath(end,3);
            x_d=0;
            y_d=0;
            z_d=0;
        end
    end



    %angles
    euler=[0;0];
    %assume no yaw for the moment and see if the controller can handle it
    euler=[euler;0];
    omegala=[0;0;0];





    s_des.pos=[x; y; z];
    s_des.vel=[x_d; y_d; z_d];
    s_des.acc=[x_dd; y_dd; z_dd];
    s_des.euler=[0;0;0];
    s_des.omegala=[0;0;0];
end
