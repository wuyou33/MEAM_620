% MEAM 620 Student Hover code
sensort.t = GetUnixTime;
if (setitM(qn)~=901) %The variable setitM(qn) tracks what type of sequence each quad is in. 
    %If the quad switches sequences, this if statement will be active.
    %This changes setitM(qn) to the current sequence type so that this code only runs once.
    initFlag = 0;
    %PUT ANY INITIALIZATION HERE
	if ~isempty(sensor.id) && sensor.isReady
                
        run bonus_EKF_init;
        setitM(qn)=901;
        des_pos = Xest_init(1:3);   % 1 by 3
        des_pos(3) = 1.5;
        
        % SETUP PARAMETERS
        mass = 0.23;
        grav = 9.81;
        kp = [3 3 3];   % proportional control gain
        kd = [2.5 2.5 2];   % derivative control gain
        t_last = sensor.t;
        
        
        posEstLog = [];
        XEstLog = [];
        visionLog = {};
        viconLog = [];
        
        % TRAJECTORY INITIALIZATIONEKF_init
        qd{qn}.vel_des = [0 0 0];
        
    end
    
    % END INTITIALIZATION
    
end %everything beyond this point runs every control loop iteration.8

%COMPUTE CONTROL HERE
if sensor.isReady
        
    % CURRENT STATE ESTIMATION

    acc = sensor.accImu;
    omega = sensor.omegaImu;
    U = [acc; omega];
    dt = sensor.t - t_last;
    
    if ~isempty(sensor.id)
        isCorrection = true;
        posEst = pos_est(sensor, tagID, K1, BRC, tagPos);
        X = posEst';
        posEstLog(end + 1, :) = posEst;
    else
        isCorrection = false;
    end
    run bonus_EKF_loop;
    
    % DATA RECORDING
    XEstLog(:, end + 1) = Xest;
    visionLog{end + 1} = sensor;
    viconLog(:, end + 1) = [qd{qn}.pos; qd{qn}.euler(1); qd{qn}.euler(2); qd{qn}.euler(3)];
    
    % Hover Control
    pos = Xest(1: 3).';
    yaw = Xest(9);
    vel = RPY2Rot(Xest(7), Xest(8), Xest(9))*(Xest(4:6));
    
    acc_des = kp .* (des_pos - pos) + kd .* (qd{qn}.vel_des - vel');
    euler_des(1) = (acc_des(1) * sin(yaw) - acc_des(2) * cos(yaw)) / grav;
    euler_des(2) = (acc_des(1) * cos(yaw) + acc_des(2) * sin(yaw)) / grav;
    euler_des(3) = 0;
    F = (acc_des(3) * mass / grav + mass) * 1000;   % u1
    trpy = [F, euler_des(1),  euler_des(2), euler_des(3)];
    t_last = sensor.t;
else
    if initFlag ~= 1
        trpy = [240, 0, 0, 0];
    end
end
%if exist('t','var')

%end