function [F, M, trpy, drpy] = controller(qd, t, trajhandle, params)

%You fill this in!

persistent euler_des_last;


interval = 0.01;

kp1 = [5000 5000 5000];   % proportional control gain
kd1 = [40 40 40];   % derivative control gain

kp2 = [20 20 20];
kd2 = [0.05 0.05 0.05];

pos = qd{1}.pos';
vel = qd{1}.vel';
euler = qd{1}.euler;
omega = qd{1}.omega';

[pos_T phi_T phi_T_d vel_T acc_T vects total_t] = trajhandle(t);  % Trajectory postion r


if t > total_t
    % Hover
    acc_des = acc_T + kp1 .* (pos_T - pos) + kd1 .* (vel_T - vel);  % Hover Control
    
    euler_des(1) = (acc_des(1) * sin(phi_T) - acc_des(2) * cos(phi_T)) / params.grav;
    euler_des(2) = (acc_des(1) * cos(phi_T) + acc_des(2) * sin(phi_T)) / params.grav;
    euler_des(3) = phi_T;
    
    omega_des(1:2) = 0;
    omega_des(3) = phi_T_d;
    
    F = acc_des(3) * params.mass + params.mass * params.grav;   % u1
    
    M = kp2 .* (euler_des - euler) + kd2 .* (omega_des - omega);  % u2
    M = M';
else
    % 3D Trajectory
    

    % Desired data

%     vect_n = vects(2, :);
%     vect_b = vects(3, :);
% 
%     % Position Control
% 
%     ep = ((pos_T - pos) * vect_n') * vect_n + ((pos_T - pos) * vect_b') * vect_b;   % 3D trajectory Control
%     ev = vel_T - vel;

    ep = pos_T - pos;
    ev = vel_T - vel;

    acc_des = kp1 .* ep + kd1 .* ev + acc_T;


    F = acc_des(3) * params.mass + params.mass * params.grav;   % u1


    % Attitude Control
    pos_des = pos_T;
    vel_des = vel_T;
    euler_des(1) = (acc_des(1) * sin(phi_T) - acc_des(2) * cos(phi_T)) / params.grav;
    euler_des(2) = (acc_des(1) * cos(phi_T) + acc_des(2) * sin(phi_T)) / params.grav;
    euler_des(3) = phi_T;


    qd{1}.pos_des = pos_des;
    qd{1}.vel_des = vel_des;
    qd{1}.euler_des = euler_des;
    
    
%     if t < 0.02
%     if isempty(euler_des_last) || sum(euler_des_last) == 0
%         vel_des
%         vel
%         acc_des
%         acc_T
%         pos_des
%         pos
%         ep
%         ev
%         F
%     end
    
    
    if isempty(euler_des_last)
        euler_des_last = euler_des;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % euler_des from last time point
%     if t > 0;
%         t = t - interval;
%     end
% 
%     [pos_T phi_T phi_T_d vel_T acc_T vects total_t] = trajhandle(t);
% 
% %     vect_n = vects(2, :);
% %     vect_b = vects(3, :);
% % 
% %     ep = ((pos_T - pos) * vect_n') * vect_n + ((pos_T - pos) * vect_b') * vect_b;
% %     ev = vel_T - vel;
% 
%     vel_T
%     vel
% 
%     ep = pos_T - pos;
%     ev = vel_T - vel;
% 
%     acc_des = kp1 .* ep + kd1 .* ev + acc_T;  % need to calculate vect_t, vect_n, vect_b, acc_des from PD control
% 
%     % acc_des = acc_T + kp1 .* (pos_T - pos) + kd1 .* (vel_T - vel);  % Hover Control
% 
%     euler_des_last(1) = (acc_des(1) * sin(phi_T) - acc_des(2) * cos(phi_T)) / params.grav;
%     euler_des_last(2) = (acc_des(1) * cos(phi_T) + acc_des(2) * sin(phi_T)) / params.grav;
%     euler_des_last(3) = phi_T;
% 
%     euler_dot_des = (euler_des - euler_des_last) / interval;
% 
%     omega_des = [cos(euler_des(2)), 0, -cos(euler_des(1)) * sin(euler_des(2));...
%         0, 1, sin(euler_des(1));...
%         sin(euler(2)), 0, cos(euler_des(1)) * cos(euler_des(2))] * euler_dot_des';
% 
%     omega_des = omega_des';
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
%     if t > 0;
%         t = t - interval;
%     end
%     
%     [pos_T phi_T phi_T_d vel_T acc_T vects total_t] = trajhandle(t);
    
%     euler_des_last(1) = (euler_des_last(1) * sin(phi_T) - euler_des_last(2) * cos(phi_T)) / params.grav;
%     euler_des_last(2) = (euler_des_last(1) * cos(phi_T) + euler_des_last(2) * sin(phi_T)) / params.grav;
%     euler_des_last(3) = phi_T;

    euler_dot_des = (euler_des - euler_des_last) / interval;

    omega_des = [cos(euler_des(2)), 0, -cos(euler_des(1)) * sin(euler_des(2));...
        0, 1, sin(euler_des(1));...
        sin(euler(2)), 0, cos(euler_des(1)) * cos(euler_des(2))] * euler_dot_des';

    omega_des = omega_des';
    
    
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    M = kp2 .* (euler_des - euler) + kd2 .* (omega_des - omega);  % u2
    M = M';
    
end

% vel
% vel_T
% F
% norm(pos - pos_T)
% acc_des
% acc_T


euler_des_last = euler_des;

trpy = 0;
drpy = 0;



end
