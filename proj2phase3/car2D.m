% Simulation of EKF-based pose etimation of a car in 2D.
% MATLAB symbolic toolbox is utilized to compute the jacobian and for value substitution
close all;
clear all;
clc;
hh = figure;

isCorrection = false;

dt = 0.1;
syms x y t vx vy w svx svy sw nx ny nt real;
X = [x;y;t];      % State/Pose
U = [vx;vy;w];    % Control/Process input
S = [svx;svy;sw]; % Process noise
D = [nx;ny;nt];   % Measurement noise

% Process
g = [x + (cos(t)*(vx+svx) - sin(t)*(vy+svy)) * dt ; ...
     y + (sin(t)*(vx+svx) + cos(t)*(vy+svy)) * dt ; ...
     t + (w+sw) * dt                             ];
% Measurement
h = [x;y;t];
% Jacobians
g = simplify(g);
g
GJ = jacobian(g, X)
LJ = jacobian(g, S)
HJ = jacobian(h, X)

% True state
X = [0;0;0];
% True const process update
U = [1;1;pi/10];

% Estimated state
Xest = [0;0;0];
% Noisy process update
Uest = [0;0;0];
% Noisy measurement
Zest = [0;0;0];
% State covariance
P = diag([0.02 0.02 0.02]);
% Process covariance
R = diag([0.1 0.1 0.1]);
% Measurement covariance
Q = diag([0.3 0.3 0.1]);

% EKF Loop simulation
for k = 1:1000
  % Process noise
  S = sqrt(R) * randn(3,1);
  % Measurement noise
  D = sqrt(Q) * randn(3,1); 
  % Noisy process
  Uest = U + S;
  % Noisy measurement
  Zest = X + D;

  % True state update
  x = X(1);
  y = X(2);
  t = X(3);
  vx = U(1);
  vy = U(2);
  w  = U(3);
  svx = 0; 
  svy = 0;
  sw  = 0;
  X = subs(g);

  % EKF Prediction
  x = Xest(1);
  y = Xest(2);
  t = Xest(3);
  vx = Uest(1);
  vy = Uest(2);
  w  = Uest(3);
  svx = 0;
  svy = 0;
  sw  = 0;
  G   = subs(GJ);
  L   = subs(LJ);
  Xest = subs(g);
  P    = G * P * G' + L * R * L';

  % EKF Correction
  if isCorrection
    x = Xest(1);
    y = Xest(2);
    t = Xest(3);
    H = subs(HJ);
    K    = P * H' * inv(H * P * H' + Q);
    Xest = Xest + K * (Zest - subs(h));
    P    = (eye(3) - K * H) * P;
  end;
  
  % Plot
  figure(hh);
  hold on;
  plot(X(1), X(2),'r.','Markersize', 10);
  plot(Zest(1), Zest(2),'g.','Markersize',10);
  plot(Xest(1), Xest(2),'b.','Markersize',10);
  hold off;
  legend('True', 'GPS', 'EKF');
  axis equal;
  grid on;
  drawnow;
end;
