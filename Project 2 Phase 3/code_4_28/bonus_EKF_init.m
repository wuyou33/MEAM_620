% This script starts up the initialization for bonus EKF.

% Written by Qiong Wang for MEAM 620 at the University of Pennsylvania.
% April 18th, 2013

% True state
X = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
% True const process update
U = [1; 1; 1; pi/10; pi/10; pi/10];

% Estimated state
Xest = [Xest_init(1:3)'; 0; 0; 0; Xest_init(4:6)'; X(10:14)]; % 14 by 1
% Noisy process update
Uest = [0; 0; 0; 0; 0; 0];  % 6 by 1
% Noisy measurement
Zest = [0; 0; 0; 0; 0; 0];
% State covariance
P = diag([0.02 0.02 0.02 0.02 0.02 0.02 0.02 0.02 0.02 0.02 0.02 0.02 0.02 0.02]);  % 14 by 14
% Process covariance
R = diag([0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1]*0.5);    % 9 by 9
% Measurement covariance
Q = diag([0.05 0.05 0.05 0.03 0.03 0.03]);