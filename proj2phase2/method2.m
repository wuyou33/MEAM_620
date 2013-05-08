close all;
% clear;
% load('studentdata1.mat');
data_size = size(data, 2);

% Camera Matrix (zero-indexed):
K = [312.554757, 0.00000000, 170.720170; ...
 0.00000000, 313.225581, 134.038406; ...
 0.00000000, 0.00000000, 1.00000000   ];
% Camera-IMU Calibration (see attached images for details):
XYZ = [-0.04, 0.0, -0.03];
Yaw = -pi/4;
Roll = -pi;
Rz = [cos(Yaw), -sin(Yaw), 0; sin(Yaw), cos(Yaw), 0; 0, 0, 1];
Rx = [1, 0, 0; 0, cos(Roll), -sin(Roll); 0, sin(Roll), cos(Roll)];

% Tag IDs:
tag_id = [57401312644, 58383764297, 59366215950, 61331119256, 63296022562, 65260925868,  1453707397,  4401062356,  9313320621, 10295772274, 14225578886, 17172933845; ...
 18155385498, 19137837151, 21102740457, 22085192110, 24050095416, 27979902028, 28962353681, 33874611946, 34857063599, 35839515252, 37804418558, 42716676823; ...
 43699128476, 46646483435, 47628935088, 49593838394, 56470999965, 57453451618, 61383258230, 12312814554, 18207524472, 23119782737, 27049589349, 28032041002; ...
 29014492655, 29996944308, 37856557532, 42768815797, 46698622409, 50628429021, 56523138939, 58488042245, 61435397204, 67330107122,  6470243610, 10400050222; ...
 12364953528, 17277211793, 32013986588, 35943793200, 37908696506, 42820954771, 52645471301, 55592826260,  5539930931, 13399544155, 29118770603, 31083673909; ...
 36978383827, 37960835480, 38943287133, 44837997051, 46802900357, 49750255316, 49802394290, 57662007514,  5644208879, 20380983674, 21363435327, 27258145245; ...
 44942274999, 63608856406,  7661251159, 14538412730, 22398025954, 25345380913, 32222542484, 62678543727, 24415068234, 31292229805, 63713134354, 25449658861; ...
 33309272085, 51975853492, 62782821675, 15677281305, 45150830895, 20641678544, 21624130197, 36360904992, 45202969869, 62887099623,  6939494376,  9886849335; ...
 29535882395, 41325302231, 13868794921, 19763504839, 21728408145, 27623118063, 40447128526, 41429580179, 46341838444, 52236548362, 37551912541, 59165848907   ];

% create the matrix for the pos of tags in world frame
hold on;
for i = 1:12
    for j = 1:9
        idx = (i - 1) * 9 + j;
        if j == 1
            tag{idx}.pos = [0.152 * i + 0.152 * (i - 1), 0.152 * (j - 1) + 0.152 * (j - 1) 0;...
                0.152 * i + 0.152 * (i - 1), 0.152 * j + 0.152 * (j - 1), 0;...
                0.152 * (i - 1) + 0.152 * (i - 1), 0.152 * j + 0.152 * (j - 1), 0;...
                0.152 * (i - 1) + 0.152 * (i - 1), 0.152 * (j - 1) + 0.152 * (j - 1), 0]';
            tag{idx}.id = tag_id(j, i);
        elseif j == 4 || j == 7
            tag{idx}.pos = tag{idx - 1}.pos + [0, 0.33, 0; 0, 0.33, 0; 0, 0.33, 0; 0, 0.33, 0]';
            tag{idx}.id = tag_id(j, i);
        else
            tag{idx}.pos = tag{idx - 1}.pos + [0, 0.304, 0; 0, 0.304, 0; 0, 0.304, 0; 0, 0.304, 0]';
            tag{idx}.id = tag_id(j, i);
        end
    end
end

count = 1;
for i = 1: data_size
    s = data{i};
    if ~isempty(s.id)
        id_size = size(s.id, 2);
        p_final = [];
        P_final = [];
        lambda_final = [];
        A_final = zeros(8*id_size, 9);
        A = [];
        for j = 1: id_size
            % get the pos of tags under camera frame
            p1 = K\[s.p1(:, j); 1];
            p2 = K\[s.p2(:, j); 1];
            p3 = K\[s.p3(:, j); 1];
            p4 = K\[s.p4(:, j); 1];
            p = [p1, p2, p3, p4];
            % get the pos of tags under world frame
            idx = find(tag_id == s.id(j));
            P = tag{idx}.pos;            
            x = p(1, :);
            y = p(2, :);
            X = P(1, :);
            Y = P(2, :);
            for k = 1:4
                A(2*k - 1: 2*k, :) = [-X(k), 0, x(k)*X(k), -Y(k), 0, x(k)*Y(k), -1, 0, x(k);...
                                    0, -X(k), y(k)*X(k), 0, -Y(k), y(k)*Y(k), 0, -1, y(k)];
            end
            A_final(8*j - 7: 8*j, :) = A;
        end
        [U S V] = svd(A_final);
        v = sign(V(end, end))*V(:, end);
        r1 = v(1:3);
        r1 = r1/norm(v(1:3));
        r2 = v(4:6);
        r2 = r2/norm(v(1:3));
        r3 = cross(r1, r2);
        R_final = [r1, r2, r3];
        T = v(7:9);
        T_final = T/norm(v(1:3));
        % compute R and T with the lambdas we get
        xyz(:, count) = R_final\(XYZ' - T_final);
        bRw = Rz*Rx*R_final;
        [row pitch yaw] = R2rpy(bRw);
        rpy(:, count) = [row; pitch; yaw];
        time(count) = s.t;
        count = count + 1;
    end
    
end
figure(1);
plot3(xyz(1, :), xyz(2, :), xyz(3, :), 'ro');
plot3(vicon(4, :), vicon(5, :), vicon(6, :));
grid;
figure(2);
vicon_t = 88.5482:0.01:((size(vicon, 2) - 1)*0.01 + 88.5482);
subplot(3, 2, 1), hold on, plot(time, xyz(1, :), 'r'), plot(vicon_t, vicon(4, :));
subplot(3, 2, 3), hold on, plot(time, xyz(2, :), 'r'), plot(vicon_t, vicon(5, :));
subplot(3, 2, 5), hold on, plot(time, xyz(3, :), 'r'), plot(vicon_t, vicon(6, :));
subplot(3, 2, 2), hold on, plot(time, rpy(1, :), 'r'), plot(vicon_t, vicon(1, :));
subplot(3, 2, 4), hold on, plot(time, rpy(2, :), 'r'), plot(vicon_t, vicon(2, :));
subplot(3, 2, 6), hold on, plot(time, rpy(3, :), 'r'), plot(vicon_t, vicon(3, :));

% error calculation
pose = [rpy; xyz];
for i = 1:6
    vicon_pose(i, :) = interp1(vicon_t, vicon(i, :), time);
    err(i, :) = pose(i, :) - vicon_pose(i, :);
    err_std(i) = std(err(i, :), 1, 2);
end