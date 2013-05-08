clear;
clc;
map = load_map('emptyMap.txt', .1, 1, 0);
goal = [1  1 1];
start  = [5  5 5];
% start = [1.81 2.40 5.5];
% goal = [9.8 8.7 0.2];
% start = [1.1 2.1 3.1];
% goal = [9.9 8.9 5.9];

% start = [0.0  -4.9 0.2];
% goal  = [8.0  18.0 3.0];

% nargin = 3;
astar = false;

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
blocks_idx = find(blocks == 1);
empty = ~reshape(map_data(:, 4), map_size);

% Index Boundary
reachable_index_min = [start_x_index - 1, start_y_index - 1, start_z_index - 1];
reachable_index_max = [start_x_index + 1, start_y_index + 1, start_z_index + 1];

% try
%     r1(reachable_index_min(2):reachable_index_max(2), reachable_index_min(1):reachable_index_max(1), reachable_index_min(3):reachable_index_max(3)) = 1;
%     r2(reachable_index_min(2):reachable_index_max(2), reachable_index_min(1):reachable_index_max(1), reachable_index_min(3):reachable_index_max(3)) = cost;
%     r1(start_y_index, start_x_index, start_z_index) = 0;
%     parent(reachable_index_min(2):reachable_index_max(2), reachable_index_min(1):reachable_index_max(1), reachable_index_min(3):reachable_index_max(3)) = start_index;
% catch exception
    reachable_index_min = bsxfun(@max, reachable_index_min, [1 1 1]);
    reachable_index_max = bsxfun(@min, reachable_index_max, [x_size, y_size, z_size]);

    r1(reachable_index_min(2):reachable_index_max(2), reachable_index_min(1):reachable_index_max(1), reachable_index_min(3):reachable_index_max(3)) = 1;
    r2(reachable_index_min(2):reachable_index_max(2), reachable_index_min(1):reachable_index_max(1), reachable_index_min(3):reachable_index_max(3)) = ...
        cost(reachable_index_min(1) - start_x_index + 2:reachable_index_max(1) - start_x_index + 2, reachable_index_min(2) - start_y_index + 2:reachable_index_max(2) - start_y_index + 2,...
                reachable_index_min(3) - start_z_index + 2:reachable_index_max(3) - start_z_index + 2);
    r1(start_y_index, start_x_index, start_z_index) = 0;
    parent(reachable_index_min(2):reachable_index_max(2), reachable_index_min(1):reachable_index_max(1), reachable_index_min(3):reachable_index_max(3)) = start_index;
% end
% reachable_index_min = [start_x_index - 1, start_y_index - 1, start_z_index - 1];
% reachable_index_max = [start_x_index + 1, start_y_index + 1, start_z_index + 1];
% min_index = [1 1 1];
% max_index = [x_size, y_size, z_size];
% i1 = find(bsxfun(@lt, reachable_index_min, min_index) == 1);
% i2 = find(bsxfun(@gt, reachable_index_max, max_index) == 1);
% reachable_index_min(i1) = min_index(i1);
% reachable_index_max(i2) = max_index(i2);



if nargin < 4
    astar = false;
end

if astar
    
else
    v1(blocks_idx) = 1;
    
    
    while v1(goal_index) == 0 && any(r2(:) ~= inf)
        [~, idx] =  min(r2(:)); % find the min  cost element in reachable group 
        r1(idx) = 0; % visited
        v1(idx) = 1; % visited
        v2(idx) = r2(idx);
        r2(idx) = inf;


     
        % find the reachable gourp of idk
        [x_idx, y_idx, z_idx] = ind2sub(map_size, idx);
%         z_idx = floor(idx / (map_size(1) * map_size(2))) + 1;
%         x_idx = floor((idx - (z_idx - 1) * map_size(1) * map_size(2)) / y_size) + 1;
%         y_idx = idx - (z_idx - 1) * map_size(1) * map_size(2) - (x_idx - 1) * y_size;
        reachable_index_min = [x_idx - 1, y_idx - 1, z_idx - 1];
        reachable_index_max = [x_idx + 1, y_idx + 1, z_idx + 1];
        
        % check boundary
%         try
%             new_reachable_group = zeros(size(r1));
% %             new_reachable_group(reachable_index_min(2):reachable_index_max(2), reachable_index_min(1):reachable_index_max(1), reachable_index_min(3):reachable_index_max(3)) = 1;
% %             new_reachable_group(x_idx - 1: x_idx + 1, y_idx - 1: y_idx + 1, z_idx - 1: z_idx + 1) = 1;
%             new_reachable_group(reachable_index_min(1):reachable_index_max(1), reachable_index_min(2):reachable_index_max(2), reachable_index_min(3):reachable_index_max(3)) = 1;
%             
%             new_reachable_not_visited = xor(new_reachable_group, v1);
%             new_reachable = new_reachable_not_visited & (r1 ~= 1);
%             old_reachable = (r1 == 1) & new_reachable_not_visited;
%             
%             
%             r1(new_reachable(new_reachable == 1)) = 1;
%             r2(new_reachable(new_reachable == 1)) = v2(idx) + 1;
%             r2(old_reachable(old_reachable == 1)) = bsxfun(@min, r2(old_reachable(old_reachable == 1)), v2(idx) + 1);
            
%             r2(reachable_index_min(2):rechable_index_max(2), rechable_index_min(1):reachable_index_max(1), reachable_index_min(3):rechable_index_max(3)) = 1;
            
%             r1 == -1 & r1 == 
            
%         catch exception
%             min_index = [1 1 1];
%             max_index = [x_size, y_size, z_size];
%             i1 = find(bsxfun(@lt, reachable_index_min, min_index) == 1);
%             i2 = find(bsxfun(@gt, reachable_index_max, max_index) == 1);
%             reachable_index_min(i1) = min_index(i1);
%             reachable_index_max(i2) = max_index(i2);
            
            reachable_index_min = bsxfun(@max, reachable_index_min, [1 1 1]);
            reachable_index_max = bsxfun(@min, reachable_index_max, [y_size, x_size, z_size]);
            
            valid_cost = cost(reachable_index_min(1) - x_idx + 2:reachable_index_max(1) - x_idx + 2, reachable_index_min(2) - y_idx + 2:reachable_index_max(2) - y_idx + 2,...
                reachable_index_min(3) - z_idx + 2:reachable_index_max(3) - z_idx + 2) + v2(idx);
            valid_cost(v1(reachable_index_min(1):reachable_index_max(1), reachable_index_min(2):reachable_index_max(2), reachable_index_min(3):reachable_index_max(3)) == 1) = inf;
%             r1_new = zeros(size(map_size));
%             r2_new = inf(map_size);
%             new_reachable_group(reachable_index_min(2):reachable_index_max(2), reachable_index_min(1):reachable_index_max(1), reachable_index_min(3):reachable_index_max(3)) = 1;
%             r1_new(reachable_index_min(1):reachable_index_max(1), reachable_index_min(2):reachable_index_max(2), reachable_index_min(3):reachable_index_max(3)) = 1;
%             r2_new(reachable_index_min(1):reachable_index_max(1), reachable_index_min(2):reachable_index_max(2), reachable_index_min(3):reachable_index_max(3)) = valid_cost; % cost of the reachable
            
            
            
%             r2_new(v1 == 1) = inf;
%             R2 = bsxfun(@min, r2, r2_new);
            
            R2 = r2;
            R2(reachable_index_min(1):reachable_index_max(1), reachable_index_min(2):reachable_index_max(2), reachable_index_min(3):reachable_index_max(3)) = ...
                bsxfun(@min, r2(reachable_index_min(1):reachable_index_max(1), reachable_index_min(2):reachable_index_max(2), reachable_index_min(3):reachable_index_max(3)),...
                valid_cost);
            
            
            % SET PARENTS
            parent(r2 ~= R2) = idx;
            
%             [x_idx1, y_idx1, z_idx1] = ind2sub(size(valid_cost), find(r2(reachable_index_min(1):reachable_index_max(1), reachable_index_min(2):reachable_index_max(2), reachable_index_min(3):reachable_index_max(3)) ~= ...
%                 R2(reachable_index_min(1):reachable_index_max(1), reachable_index_min(2):reachable_index_max(2), reachable_index_min(3):reachable_index_max(3))));
%             parent(x_idx + x_idx1 - 2, y_idx + y_idx1 - 2, z_idx + z_idx1 - 2) = idx;
            
            
            r2 = R2;
    end
end

i = 1;
path(1) = goal_index;
while goal_index ~= start_index
    path(i + 1) = parent(goal_index);
    goal_index = parent(goal_index);
    i = i + 1;
end
hold on;
scatter3(map_data(path, 1), map_data(path, 2), map_data(path, 3), '*')
% view(3);
% axis([0, 10, 0, 10, 0, 6]);
% path = parent(parent ~= inf);
num_expanded = 0;

plot_path(map, path);