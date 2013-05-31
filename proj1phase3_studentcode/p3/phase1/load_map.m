function map = load_map(filename, xy_res, z_res, margin)
% LOAD_MAP Load a map from disk.
%  MAP = LOAD_MAP(filename, xy_res, z_res, margin).  Creates an occupancy grid
%  map where a node is considered fill if it lies within 'margin' distance of
%  on abstacle.

% Import the data 
f = fopen(filename);

line = fgetl(f);
blocks = [];
while ischar(line)
    if isempty(line) || line(1) == '#'
        line = fgetl(f);
        continue;
    elseif strcmp(line(1:8), 'boundary')
        boundary = str2num(line(9: end));
        line = fgetl(f);
    elseif strcmp(line(1:5), 'block')
        blocks = [blocks; str2num(line(6: end))];
        line = fgetl(f);
    end
    
end
fclose('all');

% Mesh 
boundary_max = [boundary(4), boundary(5), boundary(6)];
boundary_min = [boundary(1), boundary(2), boundary(3)];
x_length = boundary_max(1) - boundary_min(1);
y_length = boundary_max(2) - boundary_min(2);
z_length = boundary_max(3) - boundary_min(3);
z_res = bsxfun(@min, boundary_max(3), z_res);
[x, y, z] = meshgrid(boundary_min(1): xy_res: boundary_max(1), boundary_min(2): xy_res: boundary_max(2), boundary_min(3): z_res: boundary_max(3));

% Remove the very last edge of the matrices
if mod(x_length, xy_res) == 0
    x(end, :, :) = [];
    y(end, :, :) = [];
    z(end, :, :) = [];
end
if mod(y_length, xy_res) == 0
    x(:, end, :) = [];
    y(:, end, :) = [];
    z(:, end, :) = [];
end
if mod(z_length, z_res) == 0
    x(:, :, end) = [];
    y(:, :, end) = [];
    z(:, :, end) = [];
end


map_size = size(x);

% define the blocks in the meshed map
if ~isempty(blocks)
    blocks_num = ones(size(blocks, 1), 1);

    blocks_min = [blocks(:, 1) - margin * blocks_num, blocks(:, 2) - margin * blocks_num, blocks(:, 3) - margin * blocks_num];
    blocks_max = [blocks(:, 4) + margin * blocks_num, blocks(:, 5) + margin * blocks_num, blocks(:, 6) + margin * blocks_num];
    
    result = zeros(map_size);

    for i = 1 : size(blocks, 1)
        block = zeros(map_size);
        
        blocks_min(i, :) = bsxfun(@max, blocks_min(i, :), boundary_min);
        blocks_max(i, :) = bsxfun(@min, blocks_max(i, :), boundary_max);
        
        x_index_min = floor((blocks_min(i, 1) - boundary_min(1)) / xy_res) + 1;
        y_index_min = floor((blocks_min(i, 2) - boundary_min(2)) / xy_res) + 1;
        z_index_min = floor((blocks_min(i, 3) - boundary_min(3)) / z_res) + 1;
        
        x_index_max = ceil((blocks_max(i, 1) - boundary_min(1)) / xy_res);
        y_index_max = ceil((blocks_max(i, 2) - boundary_min(2)) / xy_res);
        z_index_max = ceil((blocks_max(i, 3) - boundary_min(3)) / z_res);
        
        block(y_index_min: y_index_max, x_index_min: x_index_max, z_index_min: z_index_max) = 1;
        
        result = block | result;

    end
    blocks_x = [blocks_min(:, 1), blocks_max(:, 1)];
    blocks_y = [blocks_min(:, 2), blocks_max(:, 2)];
    blocks_z = [blocks_min(:, 3), blocks_max(:, 3)];
    colors = blocks(:, 7:9)./255;
else
    result = zeros(map_size); 
    blocks_x = double.empty(0, 2);
    blocks_y = double.empty(0, 2);
    blocks_z = double.empty(0, 2);
    colors = double.empty(0, 3);
end

% Reshape the matrice for output
x = bsxfun(@min, x(:) + xy_res/2, boundary_max(1));
y = bsxfun(@min, y(:) + xy_res/2, boundary_max(2));
z = bsxfun(@min, z(:) + z_res/2, boundary_max(3));
occupied = result(:);



map{1} = [x, y, z, occupied];
map{2} = [xy_res, z_res, map_size, boundary_min, boundary_max];
map{3} = [blocks_x, blocks_y, blocks_z, colors];

end
