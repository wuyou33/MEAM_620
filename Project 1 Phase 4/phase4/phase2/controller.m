function [F, M, trpy, drpy] = controller(qd, t, trajhandle, params)

%You fill this in!

persistent euler_des_last;


interval = 0.01;

kp1 = [20 20 20];   % proportional control gain 200
kd1 = [5 5 5];   % derivative control gain 50

kp2 = [20 20 20];
kd2 = [0.05 0.05 0.05]*1;

pos = qd{1}.pos';
vel = qd{1}.vel';
euler = qd{1}.euler;
omega = qd{1}.omega';

[pos_T phi_T phi_T_d vel_T acc_T total_t] = trajhandle(t);  % Trajectory postion r


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
    
    
    if isempty(euler_des_last)
        euler_des_last = euler_des;
    end
    
    euler_dot_des = (euler_des - euler_des_last) / interval;

    omega_des = [cos(euler_des(2)), 0, -cos(euler_des(1)) * sin(euler_des(2));...
        0, 1, sin(euler_des(1));...
        sin(euler(2)), 0, cos(euler_des(1)) * cos(euler_des(2))] * euler_dot_des';

    omega_des = omega_des';

    M = kp2 .* (euler_des - euler) + kd2 .* (omega_des - omega);  % u2
    M = M';
    
end

euler_des_last = euler_des;

trpy = 0;
drpy = 0;

end

