close all;
clear all;
clc;
addpath('./utils','./readonly','./phase1','./phase2');

% Plan path
disp('Planning ...');
map = load_map('maps/map1.txt', 0.1, 2.0, 0.3);
start = [0.0  -4.9 0.2];
stop  = [8.0  18.0 3.0];

% start = [5,1,1];
% stop = [5,29,1];

% start = [0.5 2.5 5.5];
% stop  = [19.0 2.5 5.5];

% start = [1,5,1.5];
% stop = [9,7,1.5];

path = dijkstra(map, start, stop, 1);

% Additional init script
init_script;

% Run trajectory
trajectory = run_trajectory_readonly(start, stop, map, path);

