% EXAMPLE1 PLOTS
close all;
clear;
load('example1_0.3\DesEulSave.mat');
load('example1_0.3\DesPosSave.mat');
load('example1_0.3\trpySave.mat');
load('example1_0.3\VelSave.mat');
load('example1_0.3\ViconData.mat');
load('example_waypoints\example_waypts_1.mat');
% Vicon trajectory
figure(1);
hold on;
axis equal;
plot3(ViconData(4, :), ViconData(5, :), ViconData(6, :),'r','LineWidth', 1.5);
% Desired trajectory
initial_pos = [ViconData(4, 1); ViconData(5, 1); ViconData(6, 1)];
hover_pos = initial_pos + [0; 0; 0.2];
waypts_pos = waypts';
final_pos = [-2; -1; 1];
path1 = [initial_pos'; hover_pos'];
path2 = [hover_pos'; waypts_pos(:, 1)'];
path3 = waypts_pos';
path4 = [waypts_pos(:, end)'; final_pos'];
total_t = 200;
hold on;
des_pos1 = plot_des_traj(total_t, path1);
des_pos2 = plot_des_traj(total_t, path2);
des_pos3 = plot_des_traj(total_t, path3);
des_pos4 = plot_des_traj(total_t, path4);
des_pos = [des_pos1 des_pos2 des_pos3 des_pos4];
plot3(des_pos(1, :), des_pos(2, :), des_pos(3, :),'LineWidth', 1.5);
hold on;
title('The comparison of actual and desired position for example 1');
legend('Actual Position','Desired Position');
xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');
grid on;

% EXAMPLE2 PLOTS
clear;
load('example2_1\DesEulSave.mat');
load('example2_1\DesPosSave.mat');
load('example2_1\trpySave.mat');
load('example2_1\VelSave.mat');
load('example2_1\ViconData.mat');
load('example_waypoints\example_waypts_2.mat');
% Vicon trajectory
figure(2);
hold on;
axis equal;
plot3(ViconData(4, :), ViconData(5, :), ViconData(6, :),'r','LineWidth', 1.5);
% Desired trajectory
initial_pos = [ViconData(4, 1); ViconData(5, 1); ViconData(6, 1)];
hover_pos = initial_pos + [0; 0; 0.2];
waypts_pos = waypts';
final_pos = [-2; -1; 1];
path1 = [initial_pos'; hover_pos'];
path2 = [hover_pos'; waypts_pos(:, 1)'];
path3 = waypts_pos';
path4 = [waypts_pos(:, end)'; final_pos'];
total_t = 200;
hold on;
des_pos1 = plot_des_traj(total_t, path1);
des_pos2 = plot_des_traj(total_t, path2);
des_pos3 = plot_des_traj(total_t, path3);
des_pos4 = plot_des_traj(total_t, path4);
des_pos = [des_pos1 des_pos2 des_pos3 des_pos4];
plot3(des_pos(1, :), des_pos(2, :), des_pos(3, :),'LineWidth',1.5);
title('The comparison of actual and desired position for example 2');
legend('Actual Position','Desired Position');
xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');
grid on;

% EXAMPLE2 PLOTS(faster)
clear;
load('example2_3\DesEulSave.mat');
load('example2_3\DesPosSave.mat');
load('example2_3\trpySave.mat');
load('example2_3\VelSave.mat');
load('example2_3\ViconData.mat');
load('example_waypoints\example_waypts_2.mat');
% Vicon trajectory
figure(3);
hold on;
axis equal;
plot3(ViconData(4, :), ViconData(5, :), ViconData(6, :),'r','LineWidth', 1.5);
% Desired trajectory
initial_pos = [ViconData(4, 1); ViconData(5, 1); ViconData(6, 1)];
hover_pos = initial_pos + [0; 0; 0.2];
waypts_pos = waypts';
final_pos = [-2; -1; 1];
path1 = [initial_pos'; hover_pos'];
path2 = [hover_pos'; waypts_pos(:, 1)'];
path3 = waypts_pos';
path4 = [waypts_pos(:, end)'; final_pos'];
total_t = 200;
hold on;
des_pos1 = plot_des_traj(total_t, path1);
des_pos2 = plot_des_traj(total_t, path2);
des_pos3 = plot_des_traj(total_t, path3);
des_pos4 = plot_des_traj(total_t, path4);
des_pos = [des_pos1 des_pos2 des_pos3 des_pos4];
plot3(des_pos(1, :), des_pos(2, :), des_pos(3, :),'LineWidth',1.5);
title('The comparison of actual and desired position for example 3');
legend('Actual Position','Desired Position');
xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');
grid on;

% EXAMPLE3 PLOTS
clear;
load('example3_1\DesEulSave.mat');
load('example3_1\DesPosSave.mat');
load('example3_1\trpySave.mat');
load('example3_1\VelSave.mat');
load('example3_1\ViconData.mat');
load('example_waypoints\example_waypts_3.mat');
% Vicon trajectory
figure(4);
hold on;
axis equal;
plot3(ViconData(4, :), ViconData(5, :), ViconData(6, :),'r','LineWidth', 1.5);
% Desired trajectory
initial_pos = [ViconData(4, 1); ViconData(5, 1); ViconData(6, 1)];
hover_pos = initial_pos + [0; 0; 0.2];
waypts_pos = waypts';
final_pos = [-2; -1; 1];
path1 = [initial_pos'; hover_pos'];
path2 = [hover_pos'; waypts_pos(:, 1)'];
path3 = waypts_pos';
path4 = [waypts_pos(:, end)'; final_pos'];
total_t = 200;
hold on;
des_pos1 = plot_des_traj(total_t, path1);
des_pos2 = plot_des_traj(total_t, path2);
des_pos3 = plot_des_traj(total_t, path3);
des_pos4 = plot_des_traj(total_t, path4);
des_pos = [des_pos1 des_pos2 des_pos3 des_pos4];
plot3(des_pos(1, :), des_pos(2, :), des_pos(3, :),'LineWidth',1.5);
title('The comparison of actual and desired position for example 2 with faster acceleration');
legend('Actual Position','Desired Position');
xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');
grid on;
