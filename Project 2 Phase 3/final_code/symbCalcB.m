% if ~exist('g')
grav = 9.81;
syms ax ay az x y z phi theta psi xd yd zd wx wy wz;
syms sax say saz swx swy swz sbax sbay sbaz dt; % nx ny nz nphi ntheta npsi
syms bax bay baz bphi btheta real;

X = [x; y; z; xd; yd; zd; phi; theta; psi;...
    bax; bay; baz; bphi; btheta];                    % State/Pose
U = [ax; ay; az; wx; wy; wz];                         % Control/Process input
S = [sax; say; saz; swx; swy; swz; sbax; sbay; sbaz]; % Process noise
% D = [nx; ny; nz; nphi; ntheta; npsi];                 % Measurement noise
Rot = RPY2Rot(phi, theta, psi);                       % Rotation matrix
Xdd = Rot * (U(1:3) - X(10:12) - S(1:3)) - [0; 0; grav];          % Second Derivative
% X(4:6) = Xdd .* dt;

% Process
g = [X(1:3) + X(4:6).* dt + Xdd.* dt^2/2;
    X(4:6) + Xdd.* dt;
    Rot2RPY(Rot*RPY2Rot((wx + swx)*dt, (wy + swy)*dt, (wz + swz)*dt)).';
    X(10:12) +  S(1:3).* dt;
    X(13:14)];

% Measurement
h = [x; y; z; phi; theta; psi] + [0; 0; 0; bphi; btheta; 0];
% Jacobians
g = simplify(g);
GJ = jacobian(g, X);
LJ = jacobian(g, S);
HJ = jacobian(h, X);

g = matlabFunction(g,'file','gb');
GJ = matlabFunction(GJ,'file','GJb');
LJ = matlabFunction(LJ,'file','LJb');
h = matlabFunction(h,'file','hb');
Xdd = matlabFunction(Xdd);


% end