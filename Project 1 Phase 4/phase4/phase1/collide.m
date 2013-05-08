function [C] = collide(map, points)
% COLLIDE Test whether points collide with an obstacle in an environment.
%   C = collide(map, points).  points is an M-by-3 matrix where each
%   row is an (x, y, z) point.  C in an M-by-1 logical vector; 
%   C(i) = 1 if M(i, :) touches an obstacle and is 0 otherwise.

    global MapData;
    
    block_x = [MapData.blocks_min(:, 1), MapData.blocks_max(:, 1)];
    block_y = [MapData.blocks_min(:, 2), MapData.blocks_max(:, 2)];
    block_z = [MapData.blocks_min(:, 3), MapData.blocks_max(:, 3)];

%     block_x = map{3}(:, 1:2);
%     block_y = map{3}(:, 3:4);
%     block_z = map{3}(:, 5:6);

    C = zeros(size(points, 1), 1);
    for i = 1: size(block_x, 1)
        temp = (points(:, 1) >= block_x(i, 1)) & (points(:, 1) <= block_x(i, 2)) &...
            (points(:, 2) >= block_y(i, 1)) & (points(:, 2) <= block_y(i, 2)) &...
            (points(:, 3) >= block_z(i, 1)) & (points(:, 3) <= block_z(i, 2));
%         temp = (points(:, 1) > block_x(i, 1)) & (points(:, 1) < block_x(i, 2)) &...
%             (points(:, 2) > block_y(i, 1)) & (points(:, 2) < block_y(i, 2)) &...
%             (points(:, 3) > block_z(i, 1)) & (points(:, 3) < block_z(i, 2));
        C = temp | C;
    end
    
end
