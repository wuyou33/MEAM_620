function [F, M, trpy, drpy] = controller(qd, t, trajhandle, params)
% CONTROLLER Compute the force and moment for controlling the robot 
% t: Time
% s: Current state of the quadrotor, 13 x 1 vector
% s_des: Output from "trajectory_generator"
% F: A scalar. M is a 3 x 1 vector, specifying force and moment

% Written by Qiong Wang for MEAM 620 at the University of Pennsylvania.
% Feb.6th, 2013

%% INITIALIZATION
% VARIABLES
% Initialize persistent to record previous data to do the difference.
persistent phi_des theta_des psi_des;

% PID CONTROL PARAMETERS.
% Position control.
kp_pos = [100;100;100];
kd_vel = [7.5;7.5;7.5]; % [7.5;7.5;7.5]

% Attitude control.
kp_euler = [100,100,100]; %[800,800,800]
kd_euler = [0.5;0.5;0.095]; %[0.5;0.5;0.095]

% Simulate one quad.
qn = 1;

% The difference time.
dt = 10;

% LOAD DESIRED DATA TO VARIABLES.
% Load trajectory data.
[qd{qn}.pos_des, qd{qn}.vel_des, acc_des, psi_t] = trajhandle(t);

% Calculate the trajectory acceleration from measured position and velocity.
rdd = acc_des + kp_pos .* (qd{qn}.pos_des - qd{qn}.pos) + kd_vel .* (qd{qn}.vel_des - qd{qn}.vel);

%% COMPUTE EULER ANGLES
% Record euler angles to compute angular velocity in next part.
phi_pre = phi_des;
theta_pre = theta_des;
psi_pre = psi_des;

% Obtain euler angles through acceleration.
phi_des = 1/params.grav * (rdd(1)*sin(psi_t) - rdd(2)*cos(psi_t));
theta_des = 1/params.grav * (rdd(1)*cos(psi_t) + rdd(2)*sin(psi_t));
psi_des = psi_t;
qd{qn}.euler_des = [phi_des, theta_des, psi_des];

%% COMPUTE ANGULAR VELOCITY
% Compute the derivative of euler angles.
if isempty(phi_pre)||isempty(theta_pre)||isempty(psi_pre)
    phi_pre = 0;
    theta_pre = 0;
    psi_pre = 0;
end
phidot = (phi_des - phi_pre)/dt;
thetadot = (theta_des - theta_pre)/dt;
psidot = (psi_des - psi_pre)/dt;

% Matrix transform euler angles to angular velocity,
trans_mat = [cos(theta_des)  0   -cos(phi_des)*sin(theta_des);...
             0           1               sin(phi_des);...
             sin(theta_des)  0   cos(phi_des)*cos(theta_des)];
   
% Calculate angular angles.
qd{qn}.omega_des = trans_mat * [phidot; thetadot; psidot];

%% RIGID BODY DYNAMICS
% Representation of u1 and u2.
F = params.mass * params.grav + params.mass * rdd(3);
M = (kp_euler .* (qd{qn}.euler_des - qd{qn}.euler))' + kd_euler .* (qd{qn}.omega_des - qd{qn}.omega);
trpy = [];
drpy = [];
end