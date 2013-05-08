function [path, num_expanded] = dijkstra(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an M-by-3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path.  The first
%   row is start and the last row is goal.  If no path is found, PATH is a
%   0-by-3 matrix.  Consecutive points in PATH should not be farther
%   apart than the resolution of the map.
% 
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of points that were visited while performing the search.
% Set up
xy_res = map{2}(1);
z_res = map{2}(2);
x_size = map{2}(4);
y_size = map{2}(3);
z_size = map{2}(5);
x_min = map{2}(6);
y_min = map{2}(7);
z_min = map{2}(8);
map_data = map{1};
map_size = map{2}(3:5);
x_data = reshape(map_data(:, 1), map_size);
y_data = reshape(map_data(:, 2), map_size);
z_data = reshape(map_data(:, 3), map_size);

s = size(map{1}, 1);
visited = zeros(s, 2);
reachable = -ones(s, 2);
visited(:, 2) = -1;
reachable(:, 2) = inf;

[x y z] = meshgrid(-xy_res:xy_res:xy_res, -xy_res:xy_res:xy_res, -z_res:z_res:z_res);
cost = sqrt(x.^2 + y.^2 + z.^2);
cost(2, 2, 2) = inf;


% Initialization
start_x_index = floor((start(1) - x_min) / xy_res) + 1;
start_y_index = floor((start(2) - y_min) / xy_res) + 1;
start_z_index = floor((start(3) - z_min) / z_res) + 1;
start_index = start_y_index + (start_x_index - 1) * y_size + (start_z_index - 1) * x_size * y_size;
% start_index = sub2ind(map_size, start_x_index, start_y_index, start_z_index);

goal_x_index = floor((goal(1) - x_min) / xy_res) + 1;
goal_y_index = floor((goal(2) - y_min) / xy_res) + 1;
goal_z_index = floor((goal(3) - z_min) / z_res) + 1;
goal_index = goal_y_index + (goal_x_index - 1) * y_size + (goal_z_index - 1) * x_size * y_size;
% goal_index = sub2ind(map_size, goal_x_index, goal_y_index, goal_z_index);

visited(start_index, :) = [1, 0];
v1 = reshape(visited(:, 1), map_size);    % mark
v2 = reshape(visited(:, 2), map_size);    % cost

r1 = reshape(reachable(:, 1), map_size);  % mark
r2 = reshape(reachable(:, 2), map_size);  % cost

parent = inf(map_size);
parent(start_index) = 0;

blocks = reshape(map_data(:, 4), map_size);
% blocks_idx = find();
% empty = ~reshape(map_data(:, 4), map_size);

v1(blocks == 1) = 1;

% Index Boundary
reachable_index_min = [start_x_index - 1, start_y_index - 1, start_z_index - 1];
reachable_index_max = [start_x_index + 1, start_y_index + 1, start_z_index + 1];
reachable_index_min = bsxfun(@max, reachable_index_min, [1 1 1]);
reachable_index_max = bsxfun(@min, reachable_index_max, [x_size, y_size, z_size]);


if nargin < 4
    astar = false;
end

if astar
    est_cost = sqrt((x_data(reachable_index_min(1):reachable_index_max(1), reachable_index_min(2):reachable_index_max(2), reachable_index_min(3):reachable_index_max(3)) - goal(1)).^2 +...
            (y_data(reachable_index_min(1):reachable_index_max(1), reachable_index_min(2):reachable_index_max(2), reachable_index_min(3):reachable_index_max(3)) - goal(2)).^2 +...
            (z_data(reachable_index_min(1):reachable_index_max(1), reachable_index_min(2):reachable_index_max(2), reachable_index_min(3):reachable_index_max(3)) - goal(3)).^2);
    valid_cost = cost(reachable_index_min(2) - start_y_index + 2:reachable_index_max(2) - start_y_index + 2, reachable_index_min(1) - start_x_index + 2:reachable_index_max(1) - start_x_index + 2,...
            reachable_index_min(3) - start_z_index + 2:reachable_index_max(3) - start_z_index + 2) + v2(start_index) + est_cost;
    valid_cost(v1(reachable_index_min(2):reachable_index_max(2), reachable_index_min(1):reachable_index_max(1), reachable_index_min(3):reachable_index_max(3)) == 1) = inf;
    r2(reachable_index_min(2):reachable_index_max(2), reachable_index_min(1):reachable_index_max(1), reachable_index_min(3):reachable_index_max(3)) = valid_cost + est_cost + v2(start_index);
    parent(reachable_index_min(2):reachable_index_max(2), reachable_index_min(1):reachable_index_max(1), reachable_index_min(3):reachable_index_max(3)) = start_index;
    
    
    while v1(goal_index) == 0 && any(r2(:) ~= inf)
        [~, idx] =  min(r2(:)); % find the min  cost element in reachable group 
        r1(idx) = 0; % visited
        v1(idx) = 1; % visited
        v2(idx) = r2(idx);
        r2(idx) = inf;
   
        % find the reachable gourp of idk
        [x_idx, y_idx, z_idx] = ind2sub(map_size, idx);
        reachable_index_min = [x_idx - 1, y_idx - 1, z_idx - 1];
        reachable_index_max = [x_idx + 1, y_idx + 1, z_idx + 1];
        
        % check boundary
        reachable_index_min = bsxfun(@max, reachable_index_min, [1 1 1]);
        reachable_index_max = bsxfun(@min, reachable_index_max, [y_size, x_size, z_size]);

        est_cost = sqrt((x_data(reachable_index_min(1):reachable_index_max(1), reachable_index_min(2):reachable_index_max(2), reachable_index_min(3):reachable_index_max(3)) - goal(1)).^2 +...
            (y_data(reachable_index_min(1):reachable_index_max(1), reachable_index_min(2):reachable_index_max(2), reachable_index_min(3):reachable_index_max(3)) - goal(2)).^2 +...
            (z_data(reachable_index_min(1):reachable_index_max(1), reachable_index_min(2):reachable_index_max(2), reachable_index_min(3):reachable_index_max(3)) - goal(3)).^2);
        
        valid_cost = cost(reachable_index_min(1) - x_idx + 2:reachable_index_max(1) - x_idx + 2, reachable_index_min(2) - y_idx + 2:reachable_index_max(2) - y_idx + 2,...
            reachable_index_min(3) - z_idx + 2:reachable_index_max(3) - z_idx + 2) + v2(idx);
        valid_cost(v1(reachable_index_min(1):reachable_index_max(1), reachable_index_min(2):reachable_index_max(2), reachable_index_min(3):reachable_index_max(3)) == 1) = inf;
        
        h_cost = valid_cost + est_cost;
        
        R2 = r2;
        R2_est = r2;
        R2_est(reachable_index_min(1):reachable_index_max(1), reachable_index_min(2):reachable_index_max(2), reachable_index_min(3):reachable_index_max(3)) = ...
            r2(reachable_index_min(1):reachable_index_max(1), reachable_index_min(2):reachable_index_max(2), reachable_index_min(3):reachable_index_max(3)) + est_cost;
        R2(reachable_index_min(1):reachable_index_max(1), reachable_index_min(2):reachable_index_max(2), reachable_index_min(3):reachable_index_max(3)) = ...
            bsxfun(@min, R2_est(reachable_index_min(1):reachable_index_max(1), reachable_index_min(2):reachable_index_max(2), reachable_index_min(3):reachable_index_max(3)),...
            h_cost);


        % SET PARENTS
        parent(R2_est ~= R2) = idx;
        
        r2(reachable_index_min(1):reachable_index_max(1), reachable_index_min(2):reachable_index_max(2), reachable_index_min(3):reachable_index_max(3)) = ...
                bsxfun(@min, r2(reachable_index_min(1):reachable_index_max(1), reachable_index_min(2):reachable_index_max(2), reachable_index_min(3):reachable_index_max(3)),...
                valid_cost);
    end
else
    valid_cost = cost(reachable_index_min(2) - start_y_index + 2:reachable_index_max(2) - start_y_index + 2, reachable_index_min(1) - start_x_index + 2:reachable_index_max(1) - start_x_index + 2,...
            reachable_index_min(3) - start_z_index + 2:reachable_index_max(3) - start_z_index + 2) + v2(start_index);
    valid_cost(v1(reachable_index_min(2):reachable_index_max(2), reachable_index_min(1):reachable_index_max(1), reachable_index_min(3):reachable_index_max(3)) == 1) = inf;
    r2(reachable_index_min(2):reachable_index_max(2), reachable_index_min(1):reachable_index_max(1), reachable_index_min(3):reachable_index_max(3)) = ...
        valid_cost + v2(start_index);
    parent(reachable_index_min(2):reachable_index_max(2), reachable_index_min(1):reachable_index_max(1), reachable_index_min(3):reachable_index_max(3)) = start_index;
    while v1(goal_index) == 0 && any(r2(:) ~= inf)
        [~, idx] =  min(r2(:)); % find the min  cost element in reachable group 
        r1(idx) = 0; % visited
        v1(idx) = 1; % visited
        v2(idx) = r2(idx);
        r2(idx) = inf;

        % find the reachable gourp of idk
        [x_idx, y_idx, z_idx] = ind2sub(map_size, idx);
        reachable_index_min = [x_idx - 1, y_idx - 1, z_idx - 1];
        reachable_index_max = [x_idx + 1, y_idx + 1, z_idx + 1];
        
        % check boundary
            
        reachable_index_min = bsxfun(@max, reachable_index_min, [1 1 1]);
        reachable_index_max = bsxfun(@min, reachable_index_max, [y_size, x_size, z_size]);

        valid_cost = cost(reachable_index_min(1) - x_idx + 2:reachable_index_max(1) - x_idx + 2, reachable_index_min(2) - y_idx + 2:reachable_index_max(2) - y_idx + 2,...
            reachable_index_min(3) - z_idx + 2:reachable_index_max(3) - z_idx + 2) + v2(idx);
        valid_cost(v1(reachable_index_min(1):reachable_index_max(1), reachable_index_min(2):reachable_index_max(2), reachable_index_min(3):reachable_index_max(3)) == 1) = inf;

        R2 = r2;
        R2(reachable_index_min(1):reachable_index_max(1), reachable_index_min(2):reachable_index_max(2), reachable_index_min(3):reachable_index_max(3)) = ...
            bsxfun(@min, r2(reachable_index_min(1):reachable_index_max(1), reachable_index_min(2):reachable_index_max(2), reachable_index_min(3):reachable_index_max(3)),...
            valid_cost);


        % SET PARENTS
        parent(r2 ~= R2) = idx;

        r2 = R2;
            
    end
end

% set path
if v1(goal_index)
    i = 1;
    path(1, :) = map_data(goal_index, 1:3);
    p(1) = goal_index;
    while goal_index ~= start_index
        path(i + 1, :) = map_data(parent(goal_index), 1:3);
        p(i + 1) = parent(goal_index);
        goal_index = parent(goal_index);
        i = i + 1;
    end
    path(end, :) = start;
    path(1, :) = goal;

    path = flipud(path);
else
    path = double.empty(0, 3);
end

num_expanded = sum(v1(:)) - sum(blocks(:));

end
