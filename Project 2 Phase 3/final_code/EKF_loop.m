% EKF Loop simulation
isCorrection = true;
% Process noise
%S = sqrt(R) * randn(6,1);
% Measurement noise
%D = sqrt(Q) * randn(6,1);
% Noisy process
Uest = U;
% Noisy measurement
Zest = X;

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
G = GJ(dt,phi,psi,svx,svy,svz,swx,swy,swz,theta,vx,vy,vz,wx,wy,wz);
L = LJ(dt,phi,psi,swx,swy,swz,theta,wx,wy,wz);
Xest = g(dt,phi,psi,svx,svy,svz,swx,swy,swz,theta,vx,vy,vz,wx,wy,wz,x,y,z);

P    = G * P * G' + L * R * L';

% EKF Correction
if isCorrection
    x = Xest(1);
    y = Xest(2);
    z = Xest(3);
    phi = Xest(4);
    theta = Xest(5);
    psi = Xest(6);
%     H = subs(HJ);
    H = eye(6);
    K    = P * H'/(H * P * H' + Q);
    subs_h = h(phi,psi,theta,x,y,z);
    Xest = Xest + K * (Zest - subs_h);
    P    = (eye(6) - K * H) * P;
end;