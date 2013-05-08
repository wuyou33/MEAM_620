function [F, M, trpy, drpy] = controller(qd, t, trajhandle, params)

% the first output from trajhandle is a 3-by-1 vecotr representing desired position
% trpy = [thrust roll pitch yaw], drpy = [roll pitch yaw] change,
% qd is a cell of a struct including the fields pos, vel, euler, and omega

c = @(a) cos(a);
s = @(a) sin(a);
g=params.grav;
m=params.mass;


% current and desired state vectors
x = [qd{1}.pos; qd{1}.vel; qd{1}.euler'; qd{1}.omega];
[npos nvel nacc neuler nomegala ] = feval(trajhandle, t);
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
% e_p = (dot((npos-qd{1}.pos),nhat))*nhat+(dot((npos-qd{1}.pos),bhat))*bhat;
% e_v = (nvel-qd{1}.vel);




kd= [8;8;15;1;1;1];
kp= [20;20;40;20;20;20];

% kd= [.1;.1;.1;.1;.1;.1];
% kp= [2;2;2;2;2;2];
t;

% rdd_des=kd(1:3).*e_v+kp(1:3).*e_p+nacc;
rdd_des(1)=kd(1).*(nvel(1)-qd{1}.vel(1))+kp(1).*(npos(1)-qd{1}.pos(1))+nacc(1);
rdd_des(2)=kd(2).*(nvel(2)-qd{1}.vel(2))+kp(2).*(npos(2)-qd{1}.pos(2))+nacc(2);
rdd_des(3)=kd(3).*(nvel(3)-qd{1}.vel(3))+kp(3).*(npos(3)-qd{1}.pos(3))+nacc(3);


neuler(1) = ( rdd_des(1)*s(neuler(3))-rdd_des(2)*c(neuler(3)) )/g;
neuler(2) = ( rdd_des(1)*c(neuler(3))+rdd_des(2)*s(neuler(3)) )/g;

u1 = m*g+m*rdd_des(3);
u2(1) = kp(4).*(neuler(1)-qd{1}.euler(1))+kd(4).*(nomegala(1)-qd{1}.omega(1));
u2(2) = kp(5).*(neuler(2)-qd{1}.euler(2))+kd(5).*(nomegala(2)-qd{1}.omega(2));
u2(3) = kp(6).*(neuler(3)-qd{1}.euler(3))+kd(6).*(nomegala(3)-qd{1}.omega(3));

F = u1;
M = u2';

trpy = [];
drpy = [];
