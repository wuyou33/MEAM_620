% EKF Loop simulation
%isCorrection = true;

% Process noise
%S = sqrt(R) * randn(6,1);
% Measurement noise
%D = sqrt(Q) * randn(6,1);
% Noisy process
Uest = U;
% % Noisy measurement
Zest = X;

% EKF Prediction
x = Xest(1);
y = Xest(2);
z = Xest(3);
xd = Xest(4);
yd = Xest(5);
zd = Xest(6);
phi = Xest(7);
theta = Xest(8);
psi = Xest(9);
bax = Xest(10);
bay = Xest(11);
baz = Xest(12);
bphi = Xest(13);
btheta = Xest(14);
ax = Uest(1);
ay = Uest(2);
az  = Uest(3);
wx = Uest(4);
wy = Uest(5);
wz = Uest(6);
sax = 0;
say = 0;
saz = 0;
swx = 0;
swy = 0;
swz = 0;
sbax = 0;
sbay = 0;
sbaz = 0;
% xdd = Xdd(ax,ay,az,bax,bay,baz,phi,psi,sax,say,saz,theta);

G = GJb(ax,ay,az,bax,bay,baz,dt,phi,psi,sax,say,saz,swx,swy,swz,theta,wx,wy,wz);
L = LJb(dt,phi,psi,swx,swy,swz,theta,wx,wy,wz);
Xest = gb(ax,ay,az,bax,bay,baz,bphi,btheta,dt,phi,psi,sax,say,saz,swx,swy,swz,theta,wx,wy,wz,x,xd,y,yd,z,zd);

P    = G * P * G' + L * R * L';

% EKF Correction
if isCorrection
    x = Xest(1);
    y = Xest(2);
    z = Xest(3);
    xd = Xest(4);
    yd = Xest(5);
    zd = Xest(6);
    phi = Xest(7);
    theta = Xest(8);
    psi = Xest(9);
    bax = Xest(10);
    bay = Xest(11);
    baz = Xest(12);
    bphi = Xest(13);
    btheta = Xest(14);
    H = [[eye(3); zeros(3)], zeros(6, 3), [zeros(3); eye(3)], zeros(6, 3), [zeros(3, 2);eye(2); 0, 0]];    % 6 by 14
    K    = P * H'/(H * P * H' + Q);                     % 14 by 6
    subs_h = hb(bphi,btheta,phi,psi,theta,x,y,z);
    Xest = Xest + K * (Zest - subs_h);
    P    = (eye(14) - K * H) * P;
end;