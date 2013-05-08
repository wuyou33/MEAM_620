% MEAM 620 Student Hover code

if (setitM(qn)~=901) %The variable setitM(qn) tracks what type of sequence each quad is in. 
                     %If the quad switches sequences, this if statement will be active.
    setitM(qn)=901;  %This changes setitM(qn) to the current sequence type so that this code only runs once.
      
    %PUT ANY INITIALIZATION HERE
    run controller_init;
    
    % SETUP PARAMETERS
    mass = 0.23;
    grav = 9.81;
    kp = [5 5 5];   % proportional control gain 
    kd = [2 2 2];   % derivative control gain 
    t_start = GetUnixTime;  % initial time
    t_last = t_start;
    t = t_start;
    
    % CONTROLLER INITIALIZATION
    yaw_init = Xest_init(6);
    posEstLog = [];
    XEstLog = [];
    visionLog = {};

    % TRAJECTORY INITIALIZATION
    qd{qn}.vel_des = [0 0 0];
    des_pos = Xest_init(1:3);   % 1 by 3
    
    % END INTITIALIZATION
    
end %everything beyond this point runs every control loop iteration.8

%COMPUTE CONTROL HERE
if ~isempty(sensor.id) && sensor.isReady
    t = GetUnixTime - t_start;  % time
    
    % CURRENT STATE ESTIMATION
    posEst = pos_est(sensor, tagID, K1, BRC, tagPos);
    posEstLog(end+1,:)=posEst;
    X = posEst';
    vel = qd{qn}.vel_body;
    omega = qd{qn}.omega;
    U = [vel; omega];
    dt = t - t_last;
    run EKF_loop;
    XEstLog(:,end+1)=Xest;
    visionLog{end + 1} = sensor;
    
    % Hover Control
    shift = [-1.916, -1.159, 0];
    pos = Xest(1:3).';
    yaw = qd{qn}.euler(3);
    %vel = RPY2Rot(Xest(4), Xest(5), Xest(6))*(cross(omega, Xest(1:3)) + qd{qn}.vel_body)
    vel = qd{qn}.vel;
    acc_des = kp .* (des_pos - pos) + kd .* (qd{qn}.vel_des - vel');
    euler_des(1) = (acc_des(1) * sin(yaw) - acc_des(2) * cos(yaw)) / grav;
    euler_des(2) = (acc_des(1) * cos(yaw) + acc_des(2) * sin(yaw)) / grav;
    euler_des(3) = yaw_init;
    F = (acc_des(3) * mass / grav + mass) * 1000;   % u1
    trpy = [F, euler_des(1),  euler_des(2), euler_des(3)];
end
t_last = t;
