function [F, M] = controller(t, s, s_des)
% CONTROLLER Compute the force and moment for controlling the robot 
% t: Time
% s: Current state of the quadrotor, 13 x 1 vector
% s_des: Output from "trajectory_generator"
% F: A scalar. M is a 3 x 1 vector, specifying force and moment

global params

c = @(a) cos(a);
s = @(a) sin(a);
g=params.grav;
m=params.mass;


% current and desired state vectors
pos=s(1:3);
vel=s(4:6);

R=QuatToRot(s(7:10));
[phi theta psi]=RotToRPY_ZXY(R);
euler=[phi;theta;psi]; 
omega=s(11:13);



npos=s_des.pos;
nvel=s_des.vel;
nacc=s_des.acc;
neuler=s_des.euler;
nomegala = s_des.omegala;
x_des=[npos; nvel; neuler; nomegala];

% if ( nvel==[0;0;0] )
%     that=[0;0;0];
% else
%     that=nvel./norm(nvel);
% end
% if ( nacc==[0;0;0] )
%     nhat=[0;0;0];
% else
%     nhat=nacc./norm(nacc);
% end
% 
% bhat=cross(that,nhat);
% 
% e_p = (dot((npos-.pos),nhat))*nhat+(dot((npos-pos),bhat))*bhat;
% e_v = (nvel-vel);




kd= [10;10;12;1;1;1];
kp= [20;20;20;20;20;20];

% rdd_des=kd(1:3).*e_v+kp(1:3).*e_p+nacc;
rdd_des(1)=kd(1).*(nvel(1)-vel(1))+kp(1).*(npos(1)-pos(1))+nacc(1);
rdd_des(2)=kd(2).*(nvel(2)-vel(2))+kp(2).*(npos(2)-pos(2))+nacc(2);
rdd_des(3)=kd(3).*(nvel(3)-vel(3))+kp(3).*(npos(3)-pos(3))+nacc(3);


neuler(1) = ( rdd_des(1)*s(neuler(3))-rdd_des(2)*c(neuler(3)) )/g;
neuler(2) = ( rdd_des(1)*c(neuler(3))+rdd_des(2)*s(neuler(3)) )/g;

u1 = m*g+m*rdd_des(3);
u2(1) = kp(4).*(neuler(1)-euler(1))+kd(4).*(nomegala(1)-omega(1));
u2(2) = kp(5).*(neuler(2)-euler(2))+kd(5).*(nomegala(2)-omega(2));
u2(3) = kp(6).*(neuler(3)-euler(3))+kd(6).*(nomegala(3)-omega(3));

F = u1;
M = u2';

trpy = [];
drpy = [];
