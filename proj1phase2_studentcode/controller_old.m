function [F, M, trpy, drpy] = controller(qd, t, trajhandle, params)

%You fill this in!
% freq = 1000;    % frequency of the controller
% kp1 = [400 400 400];   % proportional control gain
% kd1 = [2 2 2];   % derivative control gain
% 
% kp2 = [400 400 400];
% kd2 = [2 2 2];

kp1 = 2;
kd1 = 0.1;
kp2 = 0.1;
kd2 = 0;

[pos_T phi_T phi_T_d vel_T acc_T vects] = trajhandle();  % Trajectory postion r

pos = qd{1}.pos';
vel = qd{1}.vel';
euler = qd{1}.euler;
omega = qd{1}.omega';

% Desired data
idx = ceil(t / 0.01);

if idx == 0;
    idx = 1;
elseif idx > size(vects, 1)
    idx = size(vects, 1);
end

vect_n = vects(idx, 4:6);
vect_b = vects(idx, 7:9);
pos_t = pos_T(idx, :);
phi_t = phi_T(idx, :);
vel_t = vel_T(idx, :);
acc_t = acc_T(idx, :);

% Position Control
vel
% ep = ((pos_t - pos) * vect_n') * vect_n + ((pos_t - pos) * vect_b') * vect_b;   % 3D trajectory Control
% ev = vel_t - vel;
% acc_des = kp1 .* ep + kd1 .* ev + acc_t;

acc_des = acc_t + kp1 .* (pos_t - pos) + kd1 .* (vel_t - vel);  % Hover Control


F = acc_des(3) * params.mass + params.mass * params.grav;   % u1
F = bsxfun(@min, F, params.maxF);
F = bsxfun(@max, F, params.minF);


% Attitude Control
pos_des = pos_t;
vel_des = vel_t;
euler_des(1) = (acc_des(1) * sin(phi_t) - acc_des(2) * cos(phi_t)) / params.grav;
euler_des(2) = (acc_des(1) * cos(phi_t) + acc_des(2) * sin(phi_t)) / params.grav;
euler_des(3) = phi_t;


qd{1}.pos_des = pos_des;
qd{1}.vel_des = vel_des;
qd{1}.euler_des = euler_des;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% euler_des from last time point
% idx = idx - 1;
% if ~idx
%     idx = 1;
% end
% 
% vect_n = vects(idx, 4:6);
% vect_b = vects(idx, 7:9);
% pos_t = pos_T(idx, :);
% phi_t = phi_T(idx, :);
% vel_t = vel_T(idx, :);
% acc_t = acc_T(idx, :);
% 
% ep = ((pos_t - pos) * vect_n') * vect_n + ((pos_t - pos) * vect_b') * vect_b;
% ev = vel_t - vel;
% 
% acc_des = kp1 .* ep + kd1 .* ev + acc_t;  % need to calculate vect_t, vect_n, vect_b, acc_des from PD control
% 
% euler_des_last(1) = (acc_des(1) * sin(phi_t) - acc_des(2) * cos(phi_t)) / params.grav;
% euler_des_last(2) = (acc_des(1) * cos(phi_t) + acc_des(2) * sin(phi_t)) / params.grav;
% euler_des_last(3) = phi_t;
% 
% euler_dot_des = (euler_des - euler_des_last) / 0.01;
% 
% omega_des = [cos(euler_des(2)), 0, -cos(euler_des(1)) * sin(euler_des(2));...
%     0, 1, sin(euler_des(1));...
%     sin(euler(2)), 0, cos(euler_des(1)) * cos(euler_des(2))] * euler_dot_des';
% omega_des = omega_des';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% omega_des = [0 0 phi_T_d(idx)];
omega_des = [0 0 0];


M = kp2 .* (euler_des - euler) + kd2 .* (omega_des - omega);  % u2
M = M';
% M = [0 0 10]';
% omega_des = [1 0 -1 1; 1 1 0 -1; 1 0 1 1; 1 -1 0 -1] * [omega_h + del_omega_F; del_omega];  % desired rotor speeds

trpy = 0;
drpy = 0;




end

