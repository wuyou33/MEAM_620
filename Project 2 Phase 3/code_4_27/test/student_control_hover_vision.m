% MEAM 620 Student Hover code

if (setitM(qn)~=901) %The variable setitM(qn) tracks what type of sequence each quad is in. 
                     %If the quad switches sequences, this if statement will be active.
    setitM(qn)=901;  %This changes setitM(qn) to the current sequence type so that this code only runs once.
    
    %PUT ANY INITIALIZATION HERE
    qd{qn}.vel_des = [0 0 0];
    
    % SETUP PARAMETERS
    mass = 0.22;
    grav = 9.81;
    kp = [3 3 2];   % proportional control gain 
    kd = [1.5 1.5 1];   % derivative control gain 
    t_start = GetUnixTime;  % initial time
    t_last = t_start;
    
    % CONTROLLER INITIALIZATION
    run controller_init;
    
    % TRAJECTORY INITIALIZATION
    qd{qn}.vel_des = [0 0 0];
    
    %END INTITIALIZATION
    
end %everything beyond this point runs every control loop iteration.8

%COMPUTE CONTROL HERE
if ~isempty(sensor.id) && sensor.isReady
    t = GetUnixTime - t_start;  % time
    
    % CURRENT STATE ESTIMATION
    posEst = pos_est(sensor, tagID, K1, BRC, tagPos);
    X = posEst;
    vel = qd{1}.vel_body;
    omega = qd{1}.omega;
    U = [vel; omega];
    dt = t - t_last;
    run EKF_loop;
    
    % Hover Control
    pos = Xest(1:3).';
    yaw = Xest(6);
    vel = qd{qn}.vel';
    acc_des = kp .* (qd{qn}.pos_des' - pos) + kd .* (qd{qn}.vel_des - vel);
    euler_des(1) = (acc_des(1) * sin(yaw) - acc_des(2) * cos(yaw)) / grav;
    euler_des(2) = (acc_des(1) * cos(yaw) + acc_des(2) * sin(yaw)) / grav;
    euler_des(3) = 0;
    F = (acc_des(3) * mass / grav + mass) * 1000;   % u1
    trpy = [F, euler_des(1),  euler_des(2), euler_des(3)];
end
t_last = t;
