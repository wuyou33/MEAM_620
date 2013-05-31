function trajectory = run_trajectory_readonly(start, stop, map, path, vis)

%Controller and trajectory generator handles
controlhandle = @controller;
trajhandle = @trajectory_generator;

% Maximum position error of the quadrotor at goal
pos_tol  = 0.05; % m
% Maximum speed of the quadrotor at goal
vel_tol  = 0.05; % m/s
% Maximum time that the quadrotor is allowed to fly
time_tol = 180;  % s
% Quadrotor model
params = nanoplus_readonly();%hummingbird_readonly(); 

% Environment figure
if nargin < 5
    vis = true;
end;
if vis
    h = figure;
    set(gcf,'Renderer','OpenGL');
    plot_path(map, path);
    drawnow;
end;

% Initialize simulation
disp('Init Simulation ...');
x0     = zeros(13,1);
phi0   = 0.0; 
theta0 = 0.0; 
psi0   = 0.0;
Rot0   = RPYtoRot_ZXY(phi0,theta0,psi0);
Quat0  = RotToQuat(Rot0);
x0(1)  = start(1); %x
x0(2)  = start(2); %y
x0(3)  = start(3); %z
x0(4)  = 0;        %xdot
x0(5)  = 0;        %ydot
x0(6)  = 0;        %zdot
x0(7)  = Quat0(1); %qw
x0(8)  = Quat0(2); %qx
x0(9)  = Quat0(3); %qy
x0(10) = Quat0(4); %qz
x0(11) = 0;        %p
x0(12) = 0;        %q
x0(13) = 0;        %r
tstep  = 0.01;     % this determines the time step at which the solution is given
vstep  = 0.05;     % visualization interval
time   = 0;        % current time
x      = x0;       % state
trajectory = zeros(time_tol / tstep, 13);
sample_cnt = 1;

% Start Simulation 
disp('Start Simulation ...');
while (1)
    
    % Run simulation for vstep
    timeint = time:tstep:time+vstep;
    [tsave,xsave] = ode45(@(t,s) quadEOM(t, s, controlhandle, trajhandle, params), timeint', x);
    x    = xsave(end,:)';
    time = time + vstep;    

    % Save trajectory
    trajectory(sample_cnt:sample_cnt+4,:) = xsave(1:5,:);
    sample_cnt = sample_cnt + 5;

    % Termination criteria
    if (norm(x(1:3) - stop') < pos_tol && norm(x(4:6)) < vel_tol) || time >= time_tol
        trajectory(sample_cnt,:) = xsave(end,:);
        break;
    end;
    
    % Plot Results
    if vis
        figure(h);
        hold on;
        plot3(x(1),x(2),x(3),'b.','Markersize',10);
        hold off;
        drawnow;
    end;

end;

% Cut trajectory to appropriate length
trajectory = trajectory(1:sample_cnt,:);
