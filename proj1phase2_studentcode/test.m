clear;
clc;
T = [0 0 0 1 1 1]';

x = [0; 0; 1/3; 2/3; 1];
y = [0; 2 * sind(45); 0; -2 * sind(45); 0];
z = [0; 2 * cosd(45); 4 * cosd(45); 2 * cosd(45); 0];
points{1} = x;
points{2} = y;
points{3} = z;


% x = [x(1) 0 0 x(2) 0 0]';
% y = [y(1) 0 0 y(2) 0 0]';
% z = [z(1) 0 0 z(2) 0 0]';


% px = polyfit(t, x, 5);
% py = polyfit(t, y, 5);
% pz = polyfit(t, z, 5);

syms a b c d e f t;
eq = a * t^5 + b * t^4 + c * t^3 + d * t^2 + e * t + f;
eqd = diff(eq);
eqdd = diff(eqd);

for i = 1:3
    for j = 1:4
        g1 = [char(subs(eq, t, 0) - points{i}(j)), '=0'];
        g2 = [char(subs(eqd, t, 0)), '=0'];
        g3 = [char(subs(eqdd, t, 0)), '=0'];
        g4 = [char(subs(eq, t, 1) - points{i}(j + 1)), '=0'];
        g5 = [char(subs(eqd, t, 1)), '=0'];
        g6 = [char(subs(eqdd, t, 1)), '=0'];
        [a b c d e f] = solve(g1, g2, g3, g4, g5, g6);
        k{i, j} = [a b c d e f];
    end
end

for i = 1:4
    kx(i, :) = k{1, i};
    ky(i, :) = k{2, i};
    kz(i, :) = k{3, i};
end

g1 = [char(subs(eq, t, 0)), '=0'];
g2 = [char(subs(eqd, t, 0)), '=0'];
g3 = [char(subs(eqdd, t, 0)), '=0'];
g4 = [char(subs(eq, t, 10) - 2.5), '=0'];
g5 = [char(subs(eqd, t, 10)), '=0'];
g6 = [char(subs(eqdd, t, 10)), '=0'];
[a b c d e f] = solve(g1, g2, g3, g4, g5, g6);


% k = solve(