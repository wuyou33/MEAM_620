% This script accomplish the symbolic operation for the EKF loop.

% Written by Qiong Wang for MEAM 620 at the University of Pennsylvania.
% April 28th, 2013

% if ~exist('g', 'var')
syms x y z phi theta psi vx vy vz wx wy wz;
syms svx svy svz swx swy swz nx ny nz nphi ntheta npsi dt real;
X = [x; y; z; phi; theta; psi];         % State/Pose
U = [vx; vy; vz; wx; wy; wz];           % Control/Process input
S = [svx; svy; svz; swx; swy; swz];     % Process noise
% D = [nx; ny; nz; nphi; ntheta; npsi];   % Measurement noise
Rot = newRPY2Rot(phi, theta, psi);    % Rotation matrix

% Process
g = [X(1:3) + Rot*(U(1:3) + S(1:3)) * dt;
    newRot2RPY(Rot*newRPY2Rot((wx + swx)*dt, (wy + swy)*dt, (wz + swz)*dt)).'];

% Measurement
h = [x; y; z; phi; theta; psi];
% Jacobians
g = simplify(g);
GJ = jacobian(g, X);
LJ = jacobian(g, S);
HJ = jacobian(h, X);

g = matlabFunction(g,'file','g_JB');
GJ = matlabFunction(GJ,'file','GJ_JB');
LJ = matlabFunction(LJ,'file','LJ_JB');
h = matlabFunction(h,'file','h_JB');
% end