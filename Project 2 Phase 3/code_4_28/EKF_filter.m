function [Xest, P] = EKF_filter(U, X)
isCorrection = true;
% Noisy process
Uest = U;
% Noisy measurement
Zest = X;
% True state update
% x = X(1);
% y = X(2);
% z = X(3);
% phi = X(4);
% theta = X(5);
% psi = X(6);
% vx = U(1);
% vy = U(2);
% vz  = U(3);
% wx = U(4);
% wy = U(5);
% wz = U(6);
% svx = 0;
% svy = 0;
% svz = 0;
% swx = 0;
% swy = 0;
% swz = 0;
X = g_fun();

% EKF Prediction
x = Xest(1);
y = Xest(2);
z = Xest(3);
phi = Xest(4);
theta = Xest(5);
psi = Xest(6);
vx = Uest(1);
vy = Uest(2);
vz  = Uest(3);
wx = Uest(4);
wy = Uest(5);
wz = Uest(6);
svx = 0;
svy = 0;
svz = 0;
swx = 0;
swy = 0;
swz = 0;
G   = subs(GJ);
L   = subs(LJ);
Xest = subs(g);
P    = G * P * G' + L * R * L';

% EKF Correction
if isCorrection
    x = Xest(1);
    y = Xest(2);
    z = Xest(3);
    phi = Xest(4);
    theta = Xest(5);
    psi = Xest(6);
    H = subs(HJ);
    K    = P * H'/(H * P * H' + Q);
    Xest = Xest + K * (Zest - subs(h));
    P    = (eye(6) - K * H) * P;
end;

end