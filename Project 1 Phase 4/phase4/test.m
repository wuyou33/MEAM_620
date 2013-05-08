close all;
clear all;
clc;
total_t = 30;
map = load_map('maps/map2.txt', 1, 1.0, 0.3);
% start = [0.0  -4.9 0.2];
% stop  = [8.0  18.0 3.0];

start = [5.0  1 1];
stop  = [5  29 1];

% start = [1,5,1.5];
% stop = [9,7,1.5];

path = dijkstra(map, start, stop);
plot_path(map, path);
trajectory_generator([], map, path);
hold on;
for t = 0:0.01:total_t;
%     t
%     s_des = trajectory_generator(t, [], map, path);
    pos = trajectory_generator(t);
    plot3(pos(1), pos(2), pos(3), '*g');
%     pause(0.01);
end

%%
close all;
clear all;
clc;
total_t = 200;
load('matlab.mat')
plot_path(map, path);
trajectory_generator([], map, path);
hold on;
for t = 0:0.1:total_t;
%     t
%     s_des = trajectory_generator(t, [], map, path);
    s_des = trajectory_generator(t);
    plot3(s_des(1), s_des(2), s_des(3), '*g');
%     pause(0.01);
end