function [pos phi phi_d vel acc vects total_t] = circle_test(t)

%You fill this in!
speed = 0.1; % time cost from start to goal is 1/speed

total_t = 1 / speed;

if t > total_t
    pos = [5 0 2.5];
    vel = [0, 0, 0];
    acc = [0, 0, 0];
    vects = zeros(3);
    phi = pi/2;
    phi_d = 0;
    return;
end

k_theta = [ (3*pi)/25000, -(3*pi)/1000, pi/50, 0, 0, 0];
kz = [ 3/20000, -3/800, 1/40, 0, 0, 0];

p = [t^5, t^4, t^3, t^2, t^1, 1];
v = [5*t^4, 4*t^3, 3*t^2, 2*t, 1, 0];
a = [20*t^3, 12*t^2, 6*t, 2, 0, 0];



theta = sum(p .* k_theta);

x = 5 * cos(theta);
y = 5 * sin(theta);
z = sum(p .* kz);

% z = 0;

% phi = 2 * pi * speed * t + pi / 2;
phi = pi/2;

x_d = -5 * sin(theta) * sum(v .* k_theta);
y_d = 5 * cos(theta) * sum(v .* k_theta);
z_d = sum(v .* kz);

% z_d = 0;

phi_d = 2 * pi * speed;

x_dd = -5 * cos(theta) * sum(v .* k_theta) - 5 * sin(theta) * sum(a .* k_theta);
y_dd = -5 * sin(theta) * sum(v .* k_theta) + 5 * cos(theta) * sum(a .* k_theta);
z_dd = sum(a .* kz);
phi_d = 0;

pos = [x, y, z];
vel = [x_d, y_d, z_d];
acc = [x_dd, y_dd, z_dd];

vect_t = vel / norm(vel);
vect_n = acc / norm(acc);
vect_b = cross(vect_t, vect_n);
vects = [vect_t; vect_n; vect_b];

% hold on;
% plot3(x, y, z, 'r*-');
% hold off;

end