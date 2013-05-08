% MEAM 620 Student Hover code

if (setitM(qn)~=901) %The variable setitM(qn) tracks what type of sequence each quad is in. 
                     %If the quad switches sequences, this if statement will be active.
    setitM(qn)=901;  %This changes setitM(qn) to the current sequence type so that this code only runs once.
    
    %PUT ANY INITIALIZATION HERE
    
    qd{qn}.vel_des = [0 0 0];
    %qd{qn}.euler_des = qd{qn}.euler;
    

    t_start = GetUnixTime;
    
    %END INTITIALIZATION
    
end %everything beyond this point runs every control loop iteration.8

%COMPUTE CONTROL HERE
t = GetUnixTime - t_start;  % time

mass = 0.22;
grav = 9.81;

kp1 = [20 20 20];   % proportional control gain 200
kd1 = [5 5 5];   % derivative control gain 50

% kp2 = [50 50 50];
% kd2 = [0.05 0.05 0.05];euler_des

% Hover Control
phi_T = qd{qn}.euler(3);
pos = qd{qn}.pos';	% qd{qn}.pos is a 3 by 1 vector.
vel = qd{qn}.vel';
% euler = qd{qn}.euler;


acc_des = kp1 .* (qd{qn}.pos_des' - pos) + kd1 .* (qd{qn}.vel_des - vel);  % Hover Control

euler_des(1) = (acc_des(1) * sin(phi_T) - acc_des(2) * cos(phi_T)) / grav;
euler_des(2) = (acc_des(1) * cos(phi_T) + acc_des(2) * sin(phi_T)) / grav;
euler_des(3) = 0;


F = (acc_des(3) * mass / grav + mass) * 1000;   % u1


trpy = [F, euler_des(1),  euler_des(2), euler_des(3)];
