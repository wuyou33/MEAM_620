function map = load_map(filename, xy_res, z_res, margin)
% LOAD_MAP Load a map from disk.
%   MAP = LOAD_MAP(filename, xy_res, z_res, margin).  Creates an occupancy grid
%   map where a node is considered fill if it lies within 'margin' distance of
%   on abstacle.

% Written by Qiong Wang for MEAM 620 at the University of Pennsylvania.
% Feb.6th, 2013

%% INITIALIZE
blockNum = 0;
res = [xy_res xy_res z_res];

%% LOAD DATA
% Load txt file in the type of matrix.
% Initialize the dimension of block.
block = zeros(100,9);
% File Operation
fid = fopen(filename,'r');
tline = fgetl(fid);
while ischar(tline)
    if strcmp(sscanf(tline,'%s',1),'boundary')
        boundary = sscanf(tline,'%*s %f %f %f %f %f %f')';
    elseif strcmp(sscanf(tline,'%s',1),'block')
        blockNum = blockNum + 1;
        block(blockNum,:) = sscanf(tline,'%*s %f %f %f %f %f %f %f %f %f')';
    end
    tline = fgetl(fid);
end
fclose(fid);

% Define upperright and lowerleft of boundary.
map.boundary.lowerleft = boundary(1:3);
map.boundary.upperright = boundary(4:6);

% Use boundary data discretize the map.
spc = ceil((map.boundary.upperright - map.boundary.lowerleft)./res);
cols = spc(1);
rows = spc(2);
lyrs = spc(3);
dims = cols * rows * lyrs;

% Define struct functions. map.lim is to check if point is the within the  
% limit; map.discReal2Array is to discretize the real map position. 
re2ary = @(position) bsxfun(@min, bsxfun(@max,ceil(bsxfun(@rdivide, ...
                         bsxfun(@minus, position, map.boundary.lowerleft), ...
                         res)), 1), spc);

% Define the discrete space array, real grid length and function transfer 
% point from real to array within the map limitation.
grid = (map.boundary.upperright-map.boundary.lowerleft)./ spc;

% Initialize discrete 3-D map.
mapdata = zeros(spc);

% Set the blocks and colors in struct. ('patch' only uses RGB in [0 1])
for i = 1: blockNum
    map.block{i}.lowerleft = block(i,1:3);
    map.block{i}.upperright = block(i,4:6);
    map.block{i}.color = block(i,7:9)/255; 
end

% Set the block units in discrete 3D map to be 1.
for i = 1: blockNum
	urPoint = re2ary(map.block{i}.upperright + margin);
	llPoint = re2ary(map.block{i}.lowerleft - margin);
    mapdata(llPoint(1):urPoint(1), llPoint(2):urPoint(2), llPoint(3):urPoint(3)) = 1;
end

%% EXTERNAL ANONYMOUS FUNCTIONS AND VALUES
% Define a function that changes point from array to real within the map 
% limitation. In real map, gridCoordinate be # in [1 boundary.upperright], 
% but in real map, the center of grid cube is the position which path passes
% through. 
map.ary2re = @ (gridCoordinate) bsxfun(@plus, bsxfun(@times, gridCoordinate - .5, grid), map.boundary.lowerleft);

% Define struct array for neighbor array increment.
[nghbX,nghbY,nghbZ] = meshgrid(-1:1, -1:1, -1:1);
nghbAryInc = [nghbX(:),nghbY(:),nghbZ(:)];
map.nghbAryInc = nghbAryInc(any(nghbAryInc,2),:);

% Define a function that finds neighbour points for one position.
map.nghbFind = @findNghb;

%% LOAD DATA TO STRUCT FOR MAP
map.blockNum = blockNum;
map.res = res;
map.margin = margin;
map.spc = spc;
map.cols = cols;
map.rows = rows;
map.lyrs = lyrs;
map.dims = dims;
map.re2ary = re2ary;
map.map = mapdata;
end

%% SUB FUNCTION
function nghbDat = findNghb(map,positionAry)
% Find all the possible neighbors for a position.
nghbDat = bsxfun(@plus, map.nghbAryInc, positionAry);

% Delete position neighbors when they are out of the boundary of the map.
% CAUTION: Matrix(logical determination,:),log det only equals true or
% false.
nghbDat = nghbDat((nghbDat(:,1) >= 1) & (nghbDat(:,2) >= 1) & (nghbDat(:,3) >= 1) & ...
		  (nghbDat(:,1) <= map.cols) & (nghbDat(:,2) <= map.rows) & (nghbDat(:,3) <= map.lyrs), : );
nghbDat = nghbDat(~any(collide(map, map.ary2re(nghbDat)),2), :);
end