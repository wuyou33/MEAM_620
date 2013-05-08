function plot_path(map, path)
% PLOT_PATH Visualize a path through an environment
%   PLOT_PATH(map, path) creates a figure showing a path through the
%   environment.  path is an N-by-3 matrix where each row corresponds to the
%   (x, y, z) coordinates of one point along the path.
    blocks_x = map{3}(:, 1: 2);
    blocks_y = map{3}(:, 3: 4);
    blocks_z = map{3}(:, 5: 6);
    colors = map{3}(:, 7:9);
    if ~isempty(blocks_x)
        for i = 1 : size(blocks_x, 1)
            x = blocks_x(i, :);
            y = blocks_y(i, :);
            z = blocks_z(i, :);
            color = colors(i, :);
            vertices=[x([1 2 2 1 1 2 2 1]) ;
            y([1 1 2 2 1 1 2 2]) ;
            z([1 1 1 1 2 2 2 2])]';

            faces=[1 2 3 4;
            5 6 7 8;
            1 2 6 5;
            4 3 7 8;
            1 5 8 4;
            2 6 7 3];
            patch('Faces', faces, 'Vertices', vertices, 'FaceColor', color);
        end
    end
    hold on;
    plot3(path(:, 1), path(:, 2), path(:, 3), '-*');
    hold off;
end