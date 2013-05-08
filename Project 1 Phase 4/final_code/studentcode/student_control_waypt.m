% MEAM 620 Student One Waypoint code

if (setitM(qn)~=902) %The variable setitM(qn) tracks what type of sequence each quad is in. 
                     %If the quad switches sequences, this if statement will be active.
    setitM(qn)=902;  %This changes setitM(qn) to the current sequence type so that this code only runs once.
    
    %PUT ANY INITIALIZATION HERE
    %load('example_waypts_1');
    stop = seqM(qn).seq(seq_cntM(qn)).pos;
    %stop = waypts(1, :);
    %stop = qd{qn}.stop;
    path = [qd{qn}.pos'; stop];    % specify the start and end points here
    map = [];
    trajectory_generator([], map, path);
    psi = qd{qn}.euler(3);
    
    t_start = GetUnixTime;
    
    %END INTITIALIZATION

end %everything beyond this point runs every control loop iteration

%COMPUTE CONTROL HERE

t = GetUnixTime - t_start;

mass = 0.22;
grav = 9.81;

kp1 = [20 20 20];
kd1 = [5 5 5];

pos = qd{qn}.pos';
vel = qd{qn}.vel';
euler = qd{qn}.euler;

[pos_T phi_T phi_T_d vel_T acc_T total_t] = trajectory_generator(t);  % Trajectory postion r

yaw = qd{qn}.euler(3);

if t > total_t
    % Hover
    acc_des = acc_T + kp1 .* (pos_T - pos) + kd1 .* (vel_T - vel);  % Hover Control
    
    euler_des(1) = (acc_des(1) * sin(yaw) - acc_des(2) * cos(yaw)) / grav;
    euler_des(2) = (acc_des(1) * cos(yaw) + acc_des(2) * sin(yaw)) / grav;
    euler_des(3) = psi;
    
    F = (acc_des(3) * mass / grav + mass) * 1000;   % u1
    
else
    % 3D Trajectory

    ep = pos_T - pos;
    ev = vel_T - vel;

    acc_des = kp1 .* ep + kd1 .* ev + acc_T;

    F = (acc_des(3) * mass / grav + mass) * 1000;   % u1

    % Attitude Control
    pos_des = pos_T;
    vel_des = vel_T;
    euler_des(1) = (acc_des(1) * sin(yaw) - acc_des(2) * cos(yaw)) / grav;
    euler_des(2) = (acc_des(1) * cos(yaw) + acc_des(2) * sin(yaw)) / grav;
    euler_des(3) = psi;

    
end

trpy = [F, euler_des(1), euler_des(2), euler_des(3)];
