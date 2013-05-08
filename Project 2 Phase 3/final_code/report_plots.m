clear all;
close all;
load('waypt_pt1.mat');
shift = [1.38, 1.28, 0];

%% 3D plots
% close all;
% figure;
% hold on;
% plot3(viconLog(1, :) + shift(1), viconLog(2, :) + shift(2), viconLog(3, :), 'r');
% plot3(posEstLog(:, 1), posEstLog(:, 2), posEstLog(:, 3), 'b');
% plot3(XEstLog(1, :), XEstLog(2, :), XEstLog(3, :), 'g');
% axis equal;
% grid on;
%% 2D plots
for i = 1:size(visionLog, 2)
    t(i) = visionLog{i}.t;
end

% subplot(3, 1, 1), hold on, plot(t, viconLog(1, :) + shift(1), 'r'), ...
%     plot(t, posEstLog(:, 1), 'b'), plot(t, XEstLog(1, :), 'g'), ...
%     legend('Vicon', 'Pose Estimation', 'EKF');
% subplot(3, 1, 2), hold on, plot(t, viconLog(2, :) + shift(2), 'r'), plot(t, posEstLog(:, 2), 'b'), plot(t, XEstLog(2, :), 'g');
% subplot(3, 1, 3), hold on, plot(t, viconLog(3, :) + shift(3), 'r'), plot(t, posEstLog(:, 3), 'b'), plot(t, XEstLog(3, :), 'g');

% subplot(3, 1, 2), ;
% subplot(3, 1, 2), ;

% vicon(1:3, :) = viconLog(4:6, :);
vicon(1:3, :) = viconLog(4:6, :);
vicon(4:6, :) = bsxfun(@plus, viconLog(1:3, :), shift');
est = posEstLog';
EKF = XEstLog;
plot_all_new(EKF, est, vicon, t, t(1), t(end));