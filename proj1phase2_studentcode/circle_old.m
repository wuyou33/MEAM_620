function [pos phi phi_d vel acc vects] = circle()

%You fill this in!

% t_step = 0.01;
% step = 500;
% t_cost = t_step * step;
% time = 0:t_step:t_cost;

step = 100;
interval = 0.01;
theta = (0:2*pi/step:2*pi)';
height = 2.5;

x = 5*cos(theta);
y = 5*sin(theta);
z = (0:height/step:height)';

pos = [x y z];
phi = (0:2*pi/step:2*pi)' + pi/2;

vect_t = zeros(size(pos));
vect_n = zeros(size(pos));
vel = zeros(size(pos));
acc = zeros(size(pos));
phi_dot = zeros(size(pos));

for i = 2: size(pos, 1)
    vect = pos(i, :) - pos(i - 1, :);
    vect_t(i - 1, :) = vect / norm(vect);
    vel(i, :) = vect / interval;
    acc(i - 1, :) = (vel(i, :) - vel(i - 1, :)) / interval;
    vect_n(i - 1, :) = acc(i - 1, :) / norm(acc(i - 1, :));
    
    phi_d(i, :) = (phi(i, :) - phi(i - 1, :)) / interval;
end

vect_b = bsxfun(@cross, vect_t, vect_n);

vects = [vect_t, vect_n, vect_b];

% end_time = 5;
% t1 = timeint(1);
% t2 = timeint(end);
% interval = timeint(end) - timeint(1);
% t0 = t1 - interval;
% % step = 100;
% height = 2.5;
% 
% theta0 = 2 * pi * t0 / 5;
% theta1 = 2 * pi * t1 / 5;
% theta2 = 2 * pi * t2 / 5;
% 
% x1 = 5*cos(theta0);
% y1 = 5*sin(theta0);
% z1 = height / 5 * t0;
% 
% x1 = 5*cos(theta1);
% y1 = 5*sin(theta1);
% z1 = height / 5 * t1;
% 
% x2 = 5*cos(theta2);
% y2 = 5*sin(theta2);
% z2 = height / 5 * t2;
% 
% p0 = [x0 y0 z0];
% p1 = [x1 y1 z1];
% p2 = [x2 y2 z2];
% 
% phi1 = 2 * pi * t1 / 5 + pi / 2;
% % phi2 = 2 * pi * t2 / 5 + pi / 2;
% if t1 == 0
%     vect = p2 - p1;
%     v2 = vect / interval;
%     a1 = v2 / interval;
%     pos = p1;
%     vel = 0;
%     acc = a1;
%     phi = pi/2;
%     vect_t = vect / norm(vect);
%     vect_n = a1 / norm(a1);
%     vect_b = cross(vect_t, vect_n);
%     vects = [vect_t, vect_n, vect_b];
% elseif t1 < end_time
%     vect1 = p1 - p0;
%     vect2 = p2 - p1;
%     v1 = vect1 / interval;
%     v2 = vect2 / interval;
%     pos = p1;
%     vel = v1;
%     acc = (v2 - v1) / interval;
%     phi = phi1;
%     
% else
%     
%     
% end







% Tangent Vector and velocity
% vect = p2 - p1;
% vect_t = vect / norm(vect);
% v = vect / interval;





end