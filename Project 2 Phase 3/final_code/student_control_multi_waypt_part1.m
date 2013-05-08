% MEAM 620 Student Hover code

if (setitM(qn)~=903) %The variable setitM(qn) tracks what type of sequence each quad is in. 
                     %If the quad switches sequences, this if statement will be active.
    %setitM(qn)=903;  %This changes setitM(qn) to the current sequence type so that this code only runs once.
    initFlag = 0;
    %PUT ANY INITIALIZATION HERE
    if (~isempty(sensor.id) && sensor.isReady)
        run EKF_init;
        setitM(qn)=903;
        
        % SETUP PARAMETERS
        mass = 0.22;
        grav = 9.81;
        kp = [3 3 2];   % proportional control gain
        kd = [2 2 1];   % derivative control gain
        t_start = GetUnixTime;  % initial time
        t_last = t_start;
        
        posEstLog = [];
        XEstLog = [];
        visionLog = {};
        viconLog = [];
        % TRAJECTORY INITIALIZATION
        qd{qn}.vel_des = [0 0 0];
        path = seqM(qn).seq(seq_cntM(qn)).pos;
        path = [Xest_init(1:3); path];
        map = [];
        trajectory_generator_no_obstacle([], map, path);
    end
    %END INTITIALIZATION
    
end %everything beyond this point runs every control loop iteration.8

%COMPUTE CONTROL HERE
if ~isempty(sensor.id) && sensor.isReady
    t = GetUnixTime - t_start;  % time
    
    % DESIRED POSE FROM TRAJECTORY
    [pos_T, ~, ~, vel_T, acc_T, ~] = trajectory_generator_no_obstacle(t);
    
    % CURRENT STATE ESTIMATION
    posEst = pos_est(sensor, tagID, K1, BRC, tagPos);
    X = posEst';
    vel = qd{1}.vel_body;
    omega = qd{1}.omega;
    U = [vel; omega];
    dt = t - t_last;
    run EKF_loop;
    
    % DATA RECORDING
    posEstLog(end + 1, :) = posEst;
    XEstLog(:, end + 1) = Xest;
    visionLog{end + 1} = sensor;
    viconLog(:, end + 1) = [qd{qn}.pos; qd{qn}.euler(1); qd{qn}.euler(2); qd{qn}.euler(3)];
    
    % Hover Control
    pos = Xest(1:3).';
    yaw = Xest(6);
    vel = RPY2Rot(Xest(4), Xest(5), Xest(6))*(qd{qn}.vel_body);
    
    acc_des = kp .* (pos_T - pos) + kd .* (vel_T - vel') + acc_T;
    euler_des(1) = (acc_des(1) * sin(yaw) - acc_des(2) * cos(yaw)) / grav;
    euler_des(2) = (acc_des(1) * cos(yaw) + acc_des(2) * sin(yaw)) / grav;
    euler_des(3) = 0;
    F = (acc_des(3) * mass / grav + mass) * 1000;   % u1
    trpy = [F, euler_des(1),  euler_des(2), euler_des(3)];
else
    if initFlag ~= 1
        trpy = [240, 0, 0, 0];
    end
end
t_last = t;
