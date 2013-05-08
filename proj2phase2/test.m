close all;
% clear;
% load('studentdata1.mat');
data_size = size(data, 2);
% run('VLFEATROOT/toolbox/vl_setup');
% vl_version verbose;
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
img = cell(1, 907);
t = zeros(1, 907);
wRc = cell(1, 907);
wTc = cell(1, 907);
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
        t(count) = s.t;
        img{count} = single(s.img);
        img_new{count} = s.img;
        % rotation and translation from camera to world
        wRc{count} = inv(R_final);
        wTc{count} = R_final\(-T_final);
        
        count = count + 1;
    end
end
vicon_t = 88.5482:0.01:((size(vicon, 2) - 1)*0.01 + 88.5482);
dxyz = vicon(4:6, :) - [vicon(4, 1), vicon(4, 1: end - 1); vicon(5, 1), vicon(5, 1: end - 1); vicon(6, 1), vicon(6, 1: end - 1)];
for i = 1:3
    dxyz(i, :) = smooth(dxyz(i, :), 100)';
    vel(i, :) = dxyz(i, :)/0.01;
end
img_size = size(img, 2);
vomega = zeros(6, 1);
iter = 2;
for i = 2: iter
    % corner detection and matching
%     [fa, da] = vl_sift(img{i - 1}, 'peakthresh', 10);
%     [fb, db] = vl_sift(img{i}, 'peakthresh', 10);
%     [matches, scores] = vl_ubcmatch(da, db);
%     matches = matches(:, bsxfun(@lt, scores, 25^2));
    
    % corner detection and matching with SURF
    Ia = img_new{i - 1};
    Ib = img_new{i};
    [features_a, points_a] = cornerDetection(Ia);
    [features_b, points_b] = cornerDetection(Ib);
    indexPairs = matchFeatures(features_a, features_b, 'MatchThreshold', 0.8);    
    match_a = points_a(indexPairs(:, 1));
    match_b = points_b(indexPairs(:, 2));
    
    % uncalibrated
%     xa = fa(1, matches(1, :));
%     ya = fa(2, matches(1, :));
%     xb = fb(1, matches(2, :));
%     yb = fb(2, matches(2, :));
    
    xa = match_a.Location(:, 1)';
    ya = match_a.Location(:, 2)';
    xb = match_b.Location(:, 1)';
    yb = match_b.Location(:, 2)';
    
    % calibrated
    pts_a = K\[xa; ya; ones(1, size(xa, 2))];
    pts_b = K\[xb; yb; ones(1, size(xb, 2))];
    xa = pts_a(1, :);
    ya = pts_a(2, :);
    xb = pts_b(1, :);
    yb = pts_b(2, :);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % depth calculation
    dx = xb - xa;
    dy = yb - ya;
    dp = [dx; dy];
    dt = t(i) - t(i - 1);
    pd = dp/dt;
    z = -wTc{i}(3)./(wRc{i}(3, 1)*xb + wRc{i}(3, 2)*yb + wRc{i}(3, 3)); % depth
    % velocity calculation
    num = size(z, 2);
    M_final = zeros(2*num, 6);
    randpd = zeros(2*num, 1);
    for j = 1:num
        randx = xb(j);
        randy = yb(j);
        randz = z(j);
        randpd(2*j - 1:2*j, 1) = pd(:, j);
%         randx = xb(inlier_idx(j));
%         randy = yb(inlier_idx(j));
%         randz = z(inlier_idx(j));
%         randpd(2*j - 1:2*j, 1) = pd(:, inlier_idx(j));
        M = [-1/randz, 0, randx/randz, randx*randy, -(1 + randx^2), randy;...
            0, -1/randz, randy/randz, 1 + randy^2, -randx*randy, -randx];
        M_final(2*j - 1:2*j, :) = M;
    end
    vomega(:, i) = M_final\randpd;
%     vomega(1:3, i) = Rz*Rx*vomega(1:3, i) + Rz*Rx*(cross(-vomega(4:6, i), XYZ'));
    vomega(1:3, i) = wRc{i}*vomega(1:3, i) + cross(wRc{i}*vomega(4:6, i),wRc{i}*XYZ');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % calculate with ransac
    dt = t(i) - t(i - 1);
    [inliers_a, inliers_b] = ransac( pts_a, pts_b, dt, wRc{i}, wTc{i}, 100);
    dx = inliers_b(1, :) - inliers_a(1, :);
    dy = inliers_b(2, :) - inliers_a(2, :);
    dp = [dx; dy];
    pd = dp/dt;
    z = -wTc{i}(3)./(wRc{i}(3, 1)*inliers_b(1, :) + wRc{i}(3, 2)*inliers_b(2, :) + wRc{i}(3, 3)); % depth
    num = size(z, 2);
    M_final = zeros(2*num, 6);
    randpd = zeros(2*num, 1);
    for j = 1:num
        randx = inliers_b(1, j);
        randy = inliers_b(2, j);
        randz = z(j);
        randpd(2*j - 1:2*j, 1) = pd(:, j);
        M = [-1/randz, 0, randx/randz, randx*randy, -(1 + randx^2), randy;...
            0, -1/randz, randy/randz, 1 + randy^2, -randx*randy, -randx];
        M_final(2*j - 1:2*j, :) = M;
    end
    vomega(:, i) = M_final\randpd;
%     vomega(1:3, i) = Rz*Rx*vomega(1:3, i) + Rz*Rx*(cross(-vomega(4:6, i), XYZ'));
    vomega(1:3, i) = wRc{i}*vomega(1:3, i) + cross(wRc{i}*vomega(4:6, i),wRc{i}*XYZ');
    
    options.epsilon = 1e-6;
    options.P_inlier = 0.99;
    options.sigma = 0.01;
    options.est_fun = @estimate_plane;
    options.man_fun = @error_plane;
    options.mode = 'MSAC';
    options.Ps = [];
    options.notify_iters = [];
    options.min_iters = 1000;
    options.fix_seed = false;
    options.reestimate = true;
    options.stabilize = false;
    result = RANSAC([pts_a; pts_b], options);
    re_idx = find(result.CS == 1);
    X = [pts_a(1, re_idx); pts_b(1, re_idx)];
    Y = [pts_a(2, re_idx); pts_b(2, re_idx)];
    % show optical flow

% %     x = [fa(1, matches(1, :)); fb(1, matches(2, :))];
% %     y = [fa(2, matches(1, :)); fb(2, matches(2, :))];
% %     x = [points_a(indexPairs(:, 1), 1)'; points_b(indexPairs(:, 2), 1)'];
% %     y = [points_a(indexPairs(:, 1), 2)'; points_b(indexPairs(:, 2), 2)'];


    figure(2)
    x = [xa; xb];
    y = [ya; yb];
%     X = [inliers_a(1, :); inliers_b(1, :)];
%     Y = [inliers_a(2, :); inliers_b(2, :)];
    axis([-0.5462, 0.6568, -0.4279, 0.3383]);
    hold on;
    line(X, Y);
    plot(X, Y, 'o');
%     line(x, y);
%     plot(x, y, 'o');
    drawnow;
    pause(1)
    close(2)
end
subplot(3, 1, 1), hold on, plot(vicon_t, vel(1, :), 'r'), plot(t(1:iter), vomega(1, :));
subplot(3, 1, 2), hold on, plot(vicon_t, vel(2, :), 'r'), plot(t(1:iter), vomega(2, :));
subplot(3, 1, 3), hold on, plot(vicon_t, vel(3, :), 'r'), plot(t(1:iter), vomega(3, :));
% subplot(3, 1, 1), hold on, plot(vicon_t, vel(1, :), 'r');
% subplot(3, 1, 2), hold on, plot(vicon_t, vel(2, :), 'r');
% subplot(3, 1, 3), hold on, plot(vicon_t, vel(3, :), 'r');

