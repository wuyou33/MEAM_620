clear all;
close all;
load('waypt_pt1.mat');
shift = [1.38, 1.28, 0];

%% 3D plots
close all;
figure;
hold on;
plot3(viconLog(1, :) + shift(1), viconLog(2, :) + shift(2), viconLog(3, :), 'r');
plot3(posEstLog(:, 1), posEstLog(:, 2), posEstLog(:, 3), 'b');
plot3(XEstLog(1, :), XEstLog(2, :), XEstLog(3, :), 'g');
axis equal;
grid on;
%% 2D plots
for i = 1:size(visionLog, 2)
    t(i) = visionLog{i}.t;
end

vicon(1:3, :) = viconLog(4:6, :);
vicon(4:6, :) = bsxfun(@plus, viconLog(1:3, :), shift');
est = posEstLog';
EKF = XEstLog;
plot_all_new(EKF, est, vicon, t, t(1), t(end));

%% error plots
close all;
est_err(1:3, :) = est(1:3, :) - vicon(4:6, :);
est_err(4:6, :) = est(4:6, :) - vicon(1:3, :);
EKF_err(1:3, :) = EKF(1:3, :) - vicon(4:6, :);
EKF_err(4:6, :) = EKF(4:6, :) - vicon(1:3, :);
for i = 1:6
    est_std(i) = std(est_err(i, :), 1, 2);
    EKF_std(i) = std(EKF_err(i, :), 1, 2);
end
plot_all_new(EKF_err, est_err, zeros(size(vicon)), t, t(1), t(end));