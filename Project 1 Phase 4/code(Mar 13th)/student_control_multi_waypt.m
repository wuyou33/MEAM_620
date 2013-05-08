% MEAM 620 Student Multi Waypoint code

if (setitM(qn)~=903) %The variable setitM(qn) tracks what type of sequence each quad is in. 
                     %If the quad switches sequences, this if statement will be active.
    setitM(qn)=903;  %This changes setitM(qn) to the current sequence type so that this code only runs once.
    
    %PUT ANY INITIALIZATION HERE
    
%     start = seqM(qn).seq(seq_cntM(qn)).start;
%     stop = seqM(qn).seq(seq_cntM(qn)).stop;
    
%     path = [start; stop];    % specify the start and end points here
    path = seqM(qn).seq(seq_cntM(qn)).pos;
    map = [];
    
    path = [qd{qn}.pos'; path];
    trajectory_generator_no_obstacle([], map, path);
    
    t_start = GetUnixTime;
    
    psi = qd{qn}.euler(3);
    %END INTITIALIZATION
    j=0;

end %everything beyond this point runs every control loop iteration

%COMPUTE CONTROL HERE
j=j+1;
t = GetUnixTime - t_start;

mass = 0.22;
grav = 9.81;

kp1 = [20 20 20];   % proportional control gain 200
kd1 = [5 5 5];   % derivative control gain 50

%kp2 = [20 20 20];
%kd2 = [0.05 0.05 0.05];

pos = qd{qn}.pos';
vel = qd{qn}.vel';
euler = qd{qn}.euler;

[pos_T phi_T phi_T_d vel_T acc_T total_t] = trajectory_generator_no_obstacle(t);  % Trajectory postion r

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

DesPosSave(:,j,qn) = qd{qn}.pos_des;
DesVelSave(:,j,qn) = qd{qn}.vel_des;
DesEulSave(:,j,qn) = qd{qn}.euler_des;