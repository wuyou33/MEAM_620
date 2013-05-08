function plot_map(map)
% PLOT_MAP Plot 3D map from loaded map data.

% Written by Qiong Wang for MEAM 620 at the University of Pennsylvania.
% Jan.13th, 2013

%% INITIALIZE
% Load data from the struct.
blockNum = map.blockNum;
block = map.block;
boundary = map.boundary;
margin = map.margin;

% FaceMat can be the same by setting lower left point1 and upper right8.
faceMat = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];

% Set the alpha (transparency) of the margin space.
mapFaceAlpha = 0.3;

%% PLOT OF THE 3D MAP
% Load data into vertexMat.
for i = 1: blockNum
    vertexMat = find_vertex(map,i);
    % Plot the block.
    patch('Vertices',vertexMat,'Faces',faceMat,'FaceColor',block{i}.color)
end

% Load data with margin into vertexMat.
mapp.blockNum = blockNum;
for i = 1: blockNum
    mapp.block{i}.lowerleft = block{i}.lowerleft - margin; 
    mapp.block{i}.upperright = block{i}.upperright + margin; 
    vertexMat = find_vertex(mapp,i);
% Plot the block margin.
patch('Vertices',vertexMat,'Faces',faceMat,'FaceColor',block{i}.color,'FaceAlpha',mapFaceAlpha,'EdgeColor','none')
end

% Open figure 1.
figure(1);
axis equal
axis([ boundary.lowerleft(1) boundary.upperright(1)...
       boundary.lowerleft(2) boundary.upperright(2)...
       boundary.lowerleft(3) boundary.upperright(3)]);
grid on

% Title of figure.
title('MEAM 620 Phrase 3: Trajectory Generation and Control of a Quadrotor by Qiong Wang');

% Label the axes.
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
end

%% SUB FUNCTION
function vertexMat = find_vertex(map,iterNum)
% FIND_VERTEXMAT Find iall vertice for blocks.
% Load struct to variable.
block = map.block;
blockNum = map.blockNum;

% Initialization
xSideLeng = zeros(blockNum,1);
ySideLeng = zeros(blockNum,1);
vertexMat = zeros(8, 3);

% Define the side length.
xSideLeng(iterNum) = block{iterNum}.upperright(1)-block{iterNum}.lowerleft(1);
ySideLeng(iterNum) = block{iterNum}.upperright(2)-block{iterNum}.lowerleft(2);

% Define vertex and load data into vertexMat.
vertexMat(1,:) = block{iterNum}.lowerleft;
vertexMat(2,:) = block{iterNum}.lowerleft + [xSideLeng(iterNum) 0 0];
vertexMat(3,:) = block{iterNum}.lowerleft +[xSideLeng(iterNum) ySideLeng(iterNum) 0];
vertexMat(4,:) = block{iterNum}.lowerleft +[0 ySideLeng(iterNum) 0];
vertexMat(5,:) = block{iterNum}.upperright + [-xSideLeng(iterNum) -ySideLeng(iterNum) 0];
vertexMat(6,:) = block{iterNum}.upperright + [0 -ySideLeng(iterNum) 0];
vertexMat(7,:) = block{iterNum}.upperright;
vertexMat(8,:) = block{iterNum}.upperright + [-xSideLeng(iterNum) 0 0];
end


