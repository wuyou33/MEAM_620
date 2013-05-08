% This script starts up the initialization for EKF.

% Written by Qiong Wang for MEAM 620 at the University of Pennsylvania.
% April 16th, 2013

syms x y z phi theta psi vx vy vz wx wy wz;
syms svx svy svz swx swy swz nx ny nz nphi ntheta npsi dt real;
X = [x; y; z; phi; theta; psi];         % State/Pose
U = [vx; vy; vz; wx; wy; wz];           % Control/Process input
S = [svx; svy; svz; swx; swy; swz];     % Process noise
% D = [nx; ny; nz; nphi; ntheta; npsi];   % Measurement noise
Rot = RPY2Rot(phi, theta, psi);    % Rotation matrix

% Process
g = [X(1:3) + Rot*(U(1:3) + S(1:3)) * dt;
     Rot2RPY(Rot*RPY2Rot((wx + swx)*dt, (wy + swy)*dt, (wz + swz)*dt))'];

% Measurement
h = [x; y; z; phi; theta; psi];
% Jacobians
g = simplify(g);
GJ = jacobian(g, X);
LJ = jacobian(g, S);
HJ = jacobian(h, X);

g = matlabFunction(g);
GJ = matlabFunction(GJ);
LJ = matlabFunction(LJ);
h = matlabFunction(h);
% True state
X = [0; 0; 0; 0; 0; 0];
% True const process update
U = [1; 1; 1; pi/10; pi/10; pi/10];

% Estimated state
% Xest = [0; 0; 1; 0; 0; 0];
Xest = Xest_init';
% Noisy process update
Uest = [0; 0; 0; 0; 0; 0];
% Noisy measurement
Zest = [0; 0; 0; 0; 0; 0];
% State covariance
P = diag([0.002 0.002 0.002 0.002 0.002 0.002]);
% Process covariance
R = diag([0.1 0.1 0.1 0.1 0.1 0.1]);
% Measurement covariance
%Q = diag([0.0022 0.0015 0.0004 0.0020 0.0013 0.00001]);
Q = diag([0.3 0.3 0.3 0.1 0.1 0.1]);