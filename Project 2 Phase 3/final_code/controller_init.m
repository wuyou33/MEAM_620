% This script starts up the initialization for Controller.
if ~exist('g')
    %%%%%%%%%%%%%%%%%%% POSE ESTIMATION INITIALIZATION %%%%%%%%%%%%%%%%%%%%
    % Matrix of April tag ID.
%     tagID = ...
%         [57401312644, 58383764297, 59366215950, 61331119256, 63296022562, 65260925868,  1453707397,  4401062356,  9313320621, 10295772274, 14225578886, 17172933845; ...
%         18155385498, 19137837151, 21102740457, 22085192110, 24050095416, 27979902028, 28962353681, 33874611946, 34857063599, 35839515252, 37804418558, 42716676823; ...
%         43699128476, 46646483435, 47628935088, 49593838394, 56470999965, 57453451618, 61383258230, 12312814554, 18207524472, 23119782737, 27049589349, 28032041002; ...
%         29014492655, 29996944308, 37856557532, 42768815797, 46698622409, 50628429021, 56523138939, 58488042245, 61435397204, 67330107122,  6470243610, 10400050222; ...
%         12364953528, 17277211793, 32013986588, 35943793200, 37908696506, 42820954771, 52645471301, 55592826260,  5539930931, 13399544155, 29118770603, 31083673909; ...
%         36978383827, 37960835480, 38943287133, 44837997051, 46802900357, 49750255316, 49802394290, 57662007514,  5644208879, 20380983674, 21363435327, 27258145245; ...
%         44942274999, 63608856406,  7661251159, 14538412730, 22398025954, 25345380913, 32222542484, 62678543727, 24415068234, 31292229805, 63713134354, 25449658861; ...
%         33309272085, 51975853492, 62782821675, 15677281305, 45150830895, 20641678544, 21624130197, 36360904992, 45202969869, 62887099623,  6939494376,  9886849335; ...
%         29535882395, 41325302231, 13868794921, 19763504839, 21728408145, 27623118063, 40447128526, 41429580179, 46341838444, 52236548362, 37551912541, 59165848907   ];
%     
%     % Matrix of April tag Position
%     tagPos = cell(12*9, 1);
%     for i = 1:12
%         for j = 1:9
%             idx = (i - 1) * 9 + j;
%             if j == 1
%                 tagPos{idx}.pos = [0.152 * i + 0.152 * (i - 1), 0.152 * (j - 1) + 0.152 * (j - 1) 0;...
%                     0.152 * i + 0.152 * (i - 1), 0.152 * j + 0.152 * (j - 1), 0;...
%                     0.152 * (i - 1) + 0.152 * (i - 1), 0.152 * j + 0.152 * (j - 1), 0;...
%                     0.152 * (i - 1) + 0.152 * (i - 1), 0.152 * (j - 1) + 0.152 * (j - 1), 0]';
%                 tagPos{idx}.id = tagID(j, i);
%             elseif j == 4 || j == 7
%                 tagPos{idx}.pos = tagPos{idx - 1}.pos + [0, 0.33, 0; 0, 0.33, 0; 0, 0.33, 0; 0, 0.33, 0]';
%                 tagPos{idx}.id = tagID(j, i);
%             else
%                 tagPos{idx}.pos = tagPos{idx - 1}.pos + [0, 0.304, 0; 0, 0.304, 0; 0, 0.304, 0; 0, 0.304, 0]';
%                 tagPos{idx}.id = tagID(j, i);
%             end
%         end
%     end
%     
%     % Camera Parameters: Matrix (zero-indexed) and focal lengths.
%     K1 = ...
%         [312.554757, 0.00000000, 170.720170; ...
%         0.00000000, 313.225581, 134.038406; ...
%         0.00000000, 0.00000000, 1.00000000]^(-1);
%     
%     % Rotation from camera frame to robot frame.
%     BRC = [cosd(45) -cosd(45) 0; -cosd(45) -cosd(45) 0; 0 0 -1];
%     
%     %%%%%%%%%%%%%%%%%%%%%%% EKF INITIALIZATION %%%%%%%%%%%%%%%%%%%%%%%%%%
%     syms x y z phi theta psi vx vy vz wx wy wz;
%     syms svx svy svz swx swy swz nx ny nz nphi ntheta npsi dt real;
%     X = [x; y; z; phi; theta; psi];         % State/Pose
%     U = [vx; vy; vz; wx; wy; wz];           % Control/Process input
%     S = [svx; svy; svz; swx; swy; swz];     % Process noise
%     % D = [nx; ny; nz; nphi; ntheta; npsi];   % Measurement noise
%     Rot = RPY2Rot(phi, theta, psi);    % Rotation matrix
%     
%     % Process
%     g = [X(1:3) + Rot*(U(1:3) + S(1:3)) * dt;
%         Rot2RPY(Rot*RPY2Rot((wx + swx)*dt, (wy + swy)*dt, (wz + swz)*dt)).'];
%     
%     % Measurement
%     h = [x; y; z; phi; theta; psi];
%     % Jacobians
% %     g = simplify(g);
%     GJ = jacobian(g, X);
%     LJ = jacobian(g, S);
%     HJ = jacobian(h, X);
%     
%     g = matlabFunction(g);
%     GJ = matlabFunction(GJ);
%     LJ = matlabFunction(LJ);
%     h = matlabFunction(h);
    load('init.mat');
    % True state
    X = [0; 0; 0; 0; 0; 0];
    % True const process update
    U = [1; 1; 1; pi/10; pi/10; pi/10];
    % Initialize estimated state
    Xest_init = pos_est(sensor, tagID, K1, BRC, tagPos);
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
end
