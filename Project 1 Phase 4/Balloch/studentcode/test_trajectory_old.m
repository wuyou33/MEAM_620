close all;
clear all;
clc;
addpath('./utils','./readonly','./phase1','./phase2');

global MapData



% Plan path
disp('Planning ...');
map = load_map('maps/map1.txt', 0.1, 2.0, 0.3);
start = [0.0  -4.9 0.2];
stop  = [8.0  18.0 3.0];
% path = dijkstra(map, start, stop);
MapData.goal=stop;
MapData.start=start;

% save map1path.mat map1 path1;
load map1path.mat

% Additional init script
init_script;

% Run trajectory
trajectory = run_trajectory_readonly(start, stop, map, path);

% Evaluate trajectory
% evaulate_trajectory(trajectory); % Hidden to students

% Sample Plot
% figure;
% map.plot_path(path);
% hold on;
% plot3(trajectory(:,1),trajectory(:,2),trajectory(:,3),'b');
% hold off;
% drawnow;
