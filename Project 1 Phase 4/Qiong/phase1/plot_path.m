function plot_path(map, path)
% PLOT_PATH Visualize a path through an environment
%   PLOT_PATH(map, path) creates a figure showing a path through the
%   environment.  path is an N-by-3 matrix where each row corresponds to the
%   (x, y, z) coordinates of one point along the path.

% Written by Qiong Wang for MEAM 620 at the University of Pennsylvania.
% Jan.13th, 2013

%% PLOT THE MAP
plot_map(map);
hold on

%% PLOT THE PATH
plot3(path(:,1), path(:,2), path(:,3), 'y', 'LineWidth', 4);
end