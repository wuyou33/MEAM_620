function [pos_des, vel_des, acc_des] = diamond(t)
% DIAMOND outputs the desired data on a diamond trajectory.
%  A “diamond helix” with corners at (0; 0; 0), (0; 2 sin 45; 2 cos 45),
%  (0; 0; 4 cos 45), and (0;-2 sin 45; 2 cos 45) when projected into the yz
%  plane, and an x coordinate starting at 0 and ending at 1. The quadrotor
%  should start at (0; 0; 0) and end at (1; 0; 0).

% Written by Qiong Wang for MEAM 620 at the University of Pennsylvania.
% Jan.25th, 2013

%% INITIALIZATION
% PARAMETERS
% Initialize the four points of quadrotor on the diamond in yz plane.
start = [0; 0; 0];
corn1 = [0; 2*sind(45);  2*cosd(45)];
corn2 = [0; 0;           4*cosd(45)];
corn3 = [0; -2*sind(45); 2*cosd(45)];
edpnt = [0; 0; 0];

% The diamond trajectory points.
h = 1;
pnts_num = 5;
side_num = pnts_num - 1;
height = 0: h /side_num : h;
pnts = [start, corn1, corn2, corn3, edpnt]+[height; zeros(1,pnts_num); zeros(1,pnts_num)];
final  = pnts(:,end);

% Average velocity and corresponding flight time per side /units.
v_avg = 1;                      % m/s
side_leng = 2;                  % m
side_tm = side_leng / v_avg;    % s
time = linspace(0, side_tm, 10);

%% QUINTIC POLYNOMIAL TRAJECTORIES
% ================== INITIAL VARIABLES BEFORE THE LOOP ====================
% Initial velocity, and initial acceleration.
v0 = zeros(3,1); ac0 = zeros(3,1);

% Final velocity and final acceleration.
v1 = zeros(3,1); ac1 = zeros(3,1);

% Determine the current position on the diamond. 
t_tag = floor(t/side_tm)+1;

% ================================= LOOP ==================================
elsflg = 0;
switch t_tag
    case 1
        % The initial position, the final position and the current time.
        q0 = pnts(:,1); 
        q1 = pnts(:,2);
        tc = 0;       
    case 2
        % The initial position, the final position and the current time.
        q0 = pnts(:,2); 
        q1 = pnts(:,3);
        tc = side_tm;
    case 3
        % The initial position, the final position and the current time.
        q0 = pnts(:,3); 
        q1 = pnts(:,4);
        tc = 2* side_tm;
    case 4
        % The initial position, the final position and the current time.
        q0 = pnts(:,4); 
        q1 = pnts(:,5);
        tc = 3* side_tm;
    otherwise
        % The quatrotor has reached the final.
        elsflg = 1;        
end

if elsflg
    % Quadrotor reaches the final, velocity and acceleration become zero.
    pos_des = final;
    vel_des = zeros(3,1);
    acc_des = zeros(3,1);
else
    % The initial time and final time.
    t0 = time(1) + tc; 
    tf = time(end) + tc;

    % The quintic ploynomial trajectory matrix.
    M = [ 1  t0 t0^2 t0^3   t0^4    t0^5;...
          0  1  2*t0 3*t0^2 4*t0^3  5*t0^4;...
          0  0  2    6*t0   12*t0^2 20*t0^3;...
          1  tf tf^2 tf^3   tf^4    tf^5;...
          0  1  2*tf 3*tf^2 4*tf^3  5*tf^4;...
          0  0  2    6*tf   12*tf^2 20*tf^3];

    % Variable matrix.
    var = [q0'; v0'; ac0'; q1'; v1'; ac1'];

    % Coefficient matrix.
    a = M \ var;
    pos_des = a(1,:)' + a(2,:)'*t +a(3,:)'*t.^2 + a(4,:)'*t.^3 +a(5,:)'*t.^4 + a(6,:)'*t.^5;
    vel_des = a(2,:)' +2*a(3,:)'*t +3*a(4,:)'*t.^2 +4*a(5,:)'*t.^3 +5*a(6,:)'*t.^4;
    acc_des = 2*a(3,:)' + 6*a(4,:)'*t +12*a(5,:)'*t.^2 +20*a(6,:)'*t.^3;
end

end