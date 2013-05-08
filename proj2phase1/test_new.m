%%
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
Rx = [1, 0, 0; 0, cos(Roll), sin(Roll); 0, -sin(Roll), cos(Roll)];

cHb = [cos(Yaw), -sin(Yaw), 0, XYZ(1);...
    sin(Yaw), cos(Yaw), 0, XYZ(2);...
    0, 0, 1, XYZ(3);...
    0, 0, 0, 1];

bHc = inv(cHb);

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
        if j ~= 4 && j ~= 7
            tag{idx}.pos = [0.152 * i + 0.152 * (i - 1), 0.152 * (j - 1) + 0.152 * (j - 1) 0;...
                0.152 * i + 0.152 * (i - 1), 0.152 * j + 0.152 * (j - 1), 0;...
                0.152 * (i - 1) + 0.152 * (i - 1), 0.152 * j + 0.152 * (j - 1), 0;...
                0.152 * (i - 1) + 0.152 * (i - 1), 0.152 * (j - 1) + 0.152 * (j - 1), 0]';
            tag{idx}.id = tag_id(j, i);
        else
            tag{idx}.pos = tag{idx - 1}.pos + [0, 0.33, 0; 0, 0.33, 0; 0, 0.33, 0; 0, 0.33, 0]';
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
        for j = 1: id_size
            % get the pos of tags under camera frame
            p1 = K\[s.p1(:, j); 1];
            p2 = K\[s.p2(:, j); 1];
            p3 = K\[s.p3(:, j); 1];
            p4 = K\[s.p4(:, j); 1];
            p1 = p1/norm(p1);
            p2 = p2/norm(p2);
            p3 = p3/norm(p3);
            p4 = p4/norm(p4);
%             p = [p1, p2, p3, p4];
            p = [p1, p2, p3];
            p_final = [p_final, p];
            
            % get the pos of tags under world frame
            idx = find(tag_id == s.id(j));
            P = tag{idx}.pos;
            P = P(1:3, 1:3);
            P_final = [P_final, P];
            
            lambda = p3p(p, P);
            lambda_final = [lambda_final, lambda(1, :)];
            
            % compute R and T
            lambda_size = size(lambda, 1);
            err = zeros(lambda_size, 1);

        end
        Q = bsxfun(@times, p_final, lambda_final);
        [R T] = RT(Q, P_final);
        xyz(:, count) = R\(XYZ' - T);
        bRw = Rz*Rx*R;
        [row pitch yaw] = R2rpy(bRw);
        rpy(:, count) = [row; pitch; yaw];
        time(count) = s.t;
        count = count + 1;
    end
end
figure(1);
plot3(xyz(1, :), xyz(2, :), xyz(3, :), 'ro');
plot3(vicon(4, :), vicon(5, :), vicon(6, :));
figure(2);
vicon_t = 88.5482:0.01:((size(vicon, 2) - 1)*0.01 + 88.5482);
subplot(3, 1, 1), hold on, plot(time, xyz(1, :), 'r'), plot(vicon_t, vicon(4, :));
subplot(3, 1, 2), hold on, plot(time, xyz(2, :), 'r'), plot(vicon_t, vicon(5, :));
subplot(3, 1, 3), hold on, plot(time, xyz(3, :), 'r'), plot(vicon_t, vicon(6, :));
figure(3);
subplot(3, 1, 1), hold on, plot(time, rpy(1, :), 'r'), plot(vicon_t, vicon(1, :));
subplot(3, 1, 2), hold on, plot(time, rpy(2, :), 'r'), plot(vicon_t, vicon(2, :));
subplot(3, 1, 3), hold on, plot(time, rpy(3, :), 'r'), plot(vicon_t, vicon(3, :));

%% error calculation
% t_size = count - 1;
% for i = 1: t_size
%     t = time(i);
%     
%     err_idx(i) = (find(vicon_t >= t) == 1);
% end
vicon_x = interp1(vicon_t, vicon(4, :), time);
err_x = xyz(1, :) - vicon_x;
std_x = std(err_x, 1, 2);