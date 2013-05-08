% written by Daniel Mellinger
% July 1, 2011
% 
% For details of the controller see the paper:
% 
% Daniel Mellinger, Nathan Michael, and Vijay Kumar. 
% Trajectory Generation and Control for Precise Aggressive Maneuvers with Quadrotors. 
% Int. Symposium on Experimental Robotics, Dec 2010.
% 
% For more info take a look at some of the papers posted here:
% 
% https://fling.seas.upenn.edu/~dmel/wiki/index.php?n=Main.Publications

function sdot = quadEOM_readonly(t, s, F, M, params)


%************ EQUATIONS OF MOTION ************************
%Enforce maximum & minimum force
if(~isempty(params.maxF))
    F = min(F,params.maxF);
end
if(~isempty(params.minF))
    F = max(F,params.minF);
end
% Prop speeds omega2 
% omega2 = params.omega2_FM*[F;M];
% enforce a max & min prop speed
% if(~isempty(params.maxomega))
%     omega2 = min(omega2,params.maxomega^2*ones(4,1));
% end
% if(~isempty(params.minomega))
%     omega2 = max(omega2,params.minomega^2*ones(4,1));
% end
% Convert back to force & moment
% [FM] = params.FM_omega2*omega2;
% Re-assemble Control input
% F    = FM(1);
% M    = FM(2:4);

% Assign states
x = s(1);
y = s(2);
z = s(3);
xdot = s(4);
ydot = s(5);
zdot = s(6);
qW = s(7);
qX = s(8);
qY = s(9);
qZ = s(10);
p = s(11);
q = s(12);
r = s(13);
Rot = QuatToRot([qW,qX,qY,qZ]');
[phi,theta,yawangle] = RotToRPY_ZXY(Rot);
%init
sdot = zeros(13,1);
%rotation matrix from world frame to body frame
BRW = [ cos(yawangle)*cos(theta) - sin(phi)*sin(yawangle)*sin(theta), cos(theta)*sin(yawangle) + cos(yawangle)*sin(phi)*sin(theta), -cos(phi)*sin(theta);...
    -cos(phi)*sin(yawangle),  cos(phi)*cos(yawangle),  sin(phi);...
    cos(yawangle)*sin(theta) + cos(theta)*sin(phi)*sin(yawangle), sin(yawangle)*sin(theta) - cos(yawangle)*cos(theta)*sin(phi),  cos(phi)*cos(theta)];
WRB = BRW';

% Acceleration
accel = 1/params.mass *(WRB * [0;0;F] - [0;0;params.mass*params.grav]);
% Angular velocity
K_quat = 2; %this enforces the magnitude 1 constraint for the quaternion
quaterror = 1 - (qW^2 + qX^2 + qY^2 + qZ^2); 
qdot = -1/2*[0,-p,-q,-r;...
             p,0,-r,q;...
             q,r,0,-p;...
             r,-q,p,0]*[qW,qX,qY,qZ]' + K_quat*quaterror*[qW,qX,qY,qZ]';    
% Angular acceleration         
omega = [p;q;r];
pqrdot   = params.invI * (M - cross(omega,params.I*omega));         

% Assemble sdot
sdot(1)  = xdot;
sdot(2)  = ydot;
sdot(3)  = zdot;
sdot(4)  = accel(1);
sdot(5)  = accel(2);
sdot(6)  = accel(3);
sdot(7)  = qdot(1);
sdot(8)  = qdot(2);
sdot(9)  = qdot(3);
sdot(10) = qdot(4);
sdot(11) = pqrdot(1);
sdot(12) = pqrdot(2);
sdot(13) = pqrdot(3);
