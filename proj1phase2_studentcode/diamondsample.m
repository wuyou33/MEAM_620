close all;
clear;
hold on;
for t = 0:0.1:9.9;
    [pos, phi, phi_d, vel, acc, vects, total_t] = circle_test(t);
    plot3(pos(1), pos(2), pos(3), '*r');
    pause(0.01);
end