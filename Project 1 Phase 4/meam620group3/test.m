%%
close all;
clear all;
clc;
total_t = 200;
load('example_waypts_1.mat');
path = 
trajectory_generator_no_obstacle([], map, path);
hold on;
for t = 0:0.1:total_t;
    s_des = trajectory_generator_no_obstacle(t);
    plot3(s_des(1), s_des(2), s_des(3), '*g');
end
