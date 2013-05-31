
function [qd] = stateToQd(x)
%Converts qd struct used in hardware to x vector used in simulation
% x is 1 x 13 vector of state variables [pos vel quat omega]
% qd is a struct including the fields pos, vel, euler, and omega

%we are only simulating one quad
qn = 1;

%current state
qd{qn}.pos = x(1:3);
qd{qn}.vel = x(4:6);

Rot = QuatToRot(x(7:10)');
[phi,theta,yaw] = RotToRPY_ZXY(Rot);

qd{qn}.euler = [phi theta yaw];
qd{qn}.omega = x(11:13);

%desired state - these are set in controller
qd{qn}.pos_des = [];
qd{qn}.vel_des = [];
qd{qn}.euler_des = [];

end