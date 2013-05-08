function [ inliers_a_final inliers_b_final ] = ransac( pts_a, pts_b, dt, wRc, wTc, iter )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    % set ups
    num = size(pts_a, 2);
    tol = 1;
    for i = 1: iter
        % depth calculation
        xa = pts_a(1, :);
        ya = pts_a(2, :);
        xb = pts_b(1, :);
        yb = pts_b(2, :);
        dx = xb - xa;
        dy = yb - ya;
        dp = [dx; dy];
        pd = dp/dt;
        z = -wTc(3)./(wRc(3, 1)*xb + wRc(3, 2)*yb + wRc(3, 3)); % depth

        % randomly pick 3 sample points and caculate V and omega
        rand_idx = randsample(num, 3);
        M_final = zeros(6, 6);
        rand_pd = zeros(6, 1);
        for j = 1:3
            idx = rand_idx(j);
            rand_x = xb(idx);
            rand_y = yb(idx);
            rand_z = z(idx);
            rand_pd(2*j - 1:2*j, 1) = pd(:, j);
            M = [-1/rand_z, 0, rand_x/rand_z, rand_x*rand_y, -(1 + rand_x^2), rand_y;...
                0, -1/rand_z, rand_y/rand_z, 1 + rand_y^2, -rand_x*rand_y, -rand_x];
            M_final(2*j - 1:2*j, :) = M;
        end
        vo_sample = M_final\rand_pd;

        % check inliers
        inliers_a{i} = [];
        inliers_b{i} = [];
        count(i) = 3;
        for j = 1: num
            rand_x = xb(j);
            rand_y = yb(j);
            rand_z = z(j);
            M = [-1/rand_z, 0, rand_x/rand_z, rand_x*rand_y, -(1 + rand_x^2), rand_y;...
                0, -1/rand_z, rand_y/rand_z, 1 + rand_y^2, -rand_x*rand_y, -rand_x];
            pd_new = M*vo_sample;
            err = norm(pd_new - pd(:, j));
            if err <= tol
                inliers_a{i} = [inliers_a{i} pts_a(:, j)];
                inliers_b{i} = [inliers_b{i} pts_b(:, j)];
                count(i) = count(i) + 1;
            end
        end
    end
    
    % pick the best result
    idx = find(count == max(count));
    inliers_a_final = inliers_a{idx};
    inliers_b_final = inliers_b{idx};
end

