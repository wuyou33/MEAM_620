close all;
clear all;
clc;
addpath('./utils','./readonly','./phase1','./phase2');

% Plan path
disp('Planning ...');
map = load_map('maps/map3.txt', 0.1, 1.0, 0.3);
% start = [5.0  1 1];
% stop  = [5  29 1];

start = [0.5 2.5 5.5];
stop  = [19.0 2.5 5.5];

% start = [0.0  -4.9 0.2];
% stop  = [8.0  18.0 3.0];

% start = [1, 5, 1.5];
% stop = [9, 7, 1.5];

path = dijkstra(map, start, stop);
% plot_path(map, path);

% Additional init script
init_script;

% Run trajectory
trajectory = run_trajectory_readonly(start, stop, map, path);

if sum(collide(map, trajectory(1:3, :)))
    disp('collide');
end

%% map1
close all;
clear all;
clc;
addpath('./utils','./readonly','./phase1','./phase2');

% Plan path
disp('Planning ...');

% start = [5.0  1 1];
% stop  = [5  29 1];

% start = [0.5 2.5 5.5];
% stop  = [19.0 2.5 5.5];

% start = [0.0  -4.9 0.2];
% stop  = [8.0  18.0 3.0];

start = [0.0  -4.9 0.2];
stop  = [8.0  18.0 3.0];


load('matlab.mat');
% plot_path(map, path);

% Additional init script
init_script;

% Run trajectory
trajectory = run_trajectory_readonly(start, stop, map, path);

%%
close all;
clear all;
clc;
addpath('./utils','./readonly','./phase1','./phase2');

% Plan path
disp('Planning ...');

% start = [5.0  1 1];
% stop  = [5  29 1];

start = [0.5 2.5 5.5];
stop  = [19.0 2.5 5.5];

% start = [0.0  -4.9 0.2];
% stop  = [8.0  18.0 3.0];

% start = [0.0  -4.9 0.2];
% stop  = [8.0  18.0 3.0];

load('matlab3.mat');
% plot_path(map, path);

% Additional init script
init_script;

% Run trajectory
trajectory = run_trajectory_readonly(start, stop, map, path);

%%
close all;
clear all;
clc;
addpath('./utils','./readonly','./phase1','./phase2');

% Plan path
disp('Planning ...');

% start = [5.0  1 1];
% stop  = [5  29 1];

% start = [0.5 2.5 5.5];
% stop  = [19.0 2.5 5.5];

% start = [0.0  -4.9 0.2];
% stop  = [8.0  18.0 3.0];

start = [1, 5, 1.5];
stop = [9, 7, 1.5];

load('matlab4.mat');
% plot_path(map, path);

% Additional init script
init_script;

% Run trajectory
trajectory = run_trajectory_readonly(start, stop, map, path);

%%
close all;
clear all;
clc;
addpath('./utils','./readonly','./phase1','./phase2');

% Plan path
disp('Planning ...');

start = [5.0  1 1];
stop  = [5  29 1];

% start = [0.5 2.5 5.5];
% stop  = [19.0 2.5 5.5];

% start = [0.0  -4.9 0.2];
% stop  = [8.0  18.0 3.0];

% start = [1, 5, 1.5];
% stop = [9, 7, 1.5];

load('matlab2.mat');
% plot_path(map, path);

% Additional init script
init_script;

% Run trajectory
trajectory = run_trajectory_readonly(start, stop, map, path);