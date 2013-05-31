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

% set the cost from the current visited point to its neighbor
[x y z] = meshgrid(-xy_res:xy_res:xy_res, -xy_res:xy_res:xy_res, -z_res:z_res:z_res);
cost = sqrt(x.^2 + y.^2 + z.^2);
cost(2, 2, 2) = inf;

% Initialization
start_x_index = floor((start(1) - x_min) / xy_res) + 1;
start_y_index = floor((start(2) - y_min) / xy_res) + 1;
start_z_index = floor((start(3) - z_min) / z_res) + 1;
start_index = start_y_index + (start_x_index - 1) * y_size + (start_z_index - 1) * x_size * y_size;

goal_x_index = floor((goal(1) - x_min) / xy_res) + 1;
goal_y_index = floor((goal(2) - y_min) / xy_res) + 1;
goal_z_index = floor((goal(3) - z_min) / z_res) + 1;
goal_index = goal_y_index + (goal_x_index - 1) * y_size + (goal_z_index - 1) * x_size * y_size;

% visited(start_index, :) = [1, 0];
v1 = zeros(map_size);    % mark
v2 = zeros(map_size);    % cost
v1(start_index) = 1;

R = inf(map_size);  % cost

parents = inf(map_size);
parents(start_index) = 0;

blocks = reshape(map_data(:, 4), map_size); % indicates where is occupied by blocks

v1(blocks == 1) = 1;    % consider blocks the same as visited points so we will never visit them

% Check if start point is inside the block.
if v1(start_index)
    num_expanded = 0;
    path = double.empty(0, 3);
end

% Index Boundary
idx_min = [start_x_index - 1, start_y_index - 1, start_z_index - 1];
idx_max = [start_x_index + 1, start_y_index + 1, start_z_index + 1];
idx_min = bsxfun(@max, idx_min, [1 1 1]);
idx_max = bsxfun(@min, idx_max, [x_size, y_size, z_size]);


if nargin < 4
    astar = false;
end

if astar
    % Initialize for ASTAR
    valid_cost = cost(idx_min(2) - start_y_index + 2:idx_max(2) - start_y_index + 2, idx_min(1) - start_x_index + 2:idx_max(1) - start_x_index + 2,...
            idx_min(3) - start_z_index + 2:idx_max(3) - start_z_index + 2) + v2(start_index);
    valid_cost(v1(idx_min(2):idx_max(2), idx_min(1):idx_max(1), idx_min(3):idx_max(3)) == 1) = inf;
    R(idx_min(2):idx_max(2), idx_min(1):idx_max(1), idx_min(3):idx_max(3)) = valid_cost + v2(start_index);
    parents(idx_min(2):idx_max(2), idx_min(1):idx_max(1), idx_min(3):idx_max(3)) = start_index;
    
    while v1(goal_index) == 0 && any(R(:) ~= inf)
        h = zeros(size(R));
        h_idx = find(R(:) ~= inf);
        h(h_idx) = sqrt((x_data(h_idx) - goal(1)).^2 +...
            (y_data(h_idx) - goal(2)).^2 +...
            (z_data(h_idx) - goal(3)).^2);
        
        [~, idx] =  min(R(:) + h(:)); % find the min  cost element in reachable group 
        v1(idx) = 1; % visited
        v2(idx) = R(idx);
        R(idx) = inf;
   
        % find the reachable gourp of idk
        [x_idx, y_idx, z_idx] = ind2sub(map_size, idx);
        idx_min = [x_idx - 1, y_idx - 1, z_idx - 1];
        idx_max = [x_idx + 1, y_idx + 1, z_idx + 1];
        
        % check boundary          
        idx_min = bsxfun(@max, idx_min, [1 1 1]);
        idx_max = bsxfun(@min, idx_max, [y_size, x_size, z_size]);

        valid_cost = cost(idx_min(1) - x_idx + 2:idx_max(1) - x_idx + 2, idx_min(2) - y_idx + 2:idx_max(2) - y_idx + 2,...
            idx_min(3) - z_idx + 2:idx_max(3) - z_idx + 2) + v2(idx);
        valid_cost(v1(idx_min(1):idx_max(1), idx_min(2):idx_max(2), idx_min(3):idx_max(3)) == 1) = inf;

        R2 = R;
        R2(idx_min(1):idx_max(1), idx_min(2):idx_max(2), idx_min(3):idx_max(3)) = ...
            bsxfun(@min, R(idx_min(1):idx_max(1), idx_min(2):idx_max(2), idx_min(3):idx_max(3)),...
            valid_cost);

        % Record parents
        parents(R ~= R2) = idx;
        R = R2;
    end
else
    % Initialize for ASTAR
    valid_cost = cost(idx_min(2) - start_y_index + 2:idx_max(2) - start_y_index + 2, idx_min(1) - start_x_index + 2:idx_max(1) - start_x_index + 2,...
            idx_min(3) - start_z_index + 2:idx_max(3) - start_z_index + 2) + v2(start_index);
    valid_cost(v1(idx_min(2):idx_max(2), idx_min(1):idx_max(1), idx_min(3):idx_max(3)) == 1) = inf;
    R(idx_min(2):idx_max(2), idx_min(1):idx_max(1), idx_min(3):idx_max(3)) = ...
        valid_cost + v2(start_index);
    parents(idx_min(2):idx_max(2), idx_min(1):idx_max(1), idx_min(3):idx_max(3)) = start_index;
    
    while v1(goal_index) == 0 && any(R(:) ~= inf)
        [~, idx] =  min(R(:)); % find the min  cost element in reachable group 
        v1(idx) = 1; % visited
        v2(idx) = R(idx);
        R(idx) = inf;

        % find the reachable gourp of idk
        [x_idx, y_idx, z_idx] = ind2sub(map_size, idx);
        idx_min = [x_idx - 1, y_idx - 1, z_idx - 1];
        idx_max = [x_idx + 1, y_idx + 1, z_idx + 1];
        
        % check boundary
            
        idx_min = bsxfun(@max, idx_min, [1 1 1]);
        idx_max = bsxfun(@min, idx_max, [y_size, x_size, z_size]);

        valid_cost = cost(idx_min(1) - x_idx + 2:idx_max(1) - x_idx + 2, idx_min(2) - y_idx + 2:idx_max(2) - y_idx + 2,...
            idx_min(3) - z_idx + 2:idx_max(3) - z_idx + 2) + v2(idx);
        valid_cost(v1(idx_min(1):idx_max(1), idx_min(2):idx_max(2), idx_min(3):idx_max(3)) == 1) = inf;

        R2 = R;
        R2(idx_min(1):idx_max(1), idx_min(2):idx_max(2), idx_min(3):idx_max(3)) = ...
            bsxfun(@min, R(idx_min(1):idx_max(1), idx_min(2):idx_max(2), idx_min(3):idx_max(3)),...
            valid_cost);

        % Record parents
        parents(R ~= R2) = idx;
        R = R2;
    end
end

% set path
if v1(goal_index) && parents(goal_index) ~= inf
    i = 1;
    path(1, :) = map_data(goal_index, 1:3);
    while goal_index ~= start_index
        path(i + 1, :) = map_data(parents(goal_index), 1:3);
        goal_index = parents(goal_index);
        i = i + 1;
    end
    path(end, :) = start;
    path(1, :) = goal;

    path = flipud(path);
else
    path = double.empty(0, 3);  % set path as empty matrix if no valid choice is found.
end

num_expanded = sum(v1(:)) - sum(blocks(:));

end
