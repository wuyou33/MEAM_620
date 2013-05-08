function [path, num_expanded, best_cost] = dijkstra(map, start, goal, astar)
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

% Written by Qiong Wang for MEAM 620 at the University of Pennsylvania.
% Feb.th, 2013

%% DETERMINE METHOD
% If the # of input arguments equals or more than 4, then astar can be
% chosen.
if nargin < 4
    astar = false;
end

%% INITIALIZE
% Load struct to variables.
cols = map.cols;
rows = map.rows;
spc = map.spc;
dims = map.dims;
re2ary = map.re2ary;
ary2re = map.ary2re;
nghbFind = map.nghbFind;

% Discretize the start and goal. The current node is the start.
stAry = re2ary(start);
glAry = re2ary(goal);
node = stAry;

% Define a function to calculate the cost and heuristic distance between 
% points A & B.
costCal = @(nghbAry, nodeCurrent) sqrt(sum(bsxfun(@power, bsxfun(@minus,ary2re(nghbAry),ary2re(nodeCurrent)),2),2));
hrCost = @(nghbAry) sqrt(sum(bsxfun(@power, bsxfun(@minus, ary2re(nghbAry), goal), 2.0), 2));

% Define function transferring sub to index and index to sub.
s2i = @ (sub) sub(:,1)+ (sub(:,2)-1) * cols + (sub(:,3)-1) * cols * rows;
i2s = @ (index) ind2sub(spc, index);

% Initialize three space matrice: visited, reachable, parent representing
% the cost from start to each discrete point in the map.
visited = zeros(spc);
reachable = nghbFind(map,node);
rchFlag = zeros(spc);
parent = zeros(dims, 1);

% Define the cost matrix the initial value is infinite.
cost = inf(dims, 1);
cost(s2i(node)) = 0;
cost(s2i(reachable)) = costCal(reachable,node);

%% ERROR CASES
if map.map(stAry(1),stAry(2),stAry(3)) || map.map(glAry(1),glAry(2),glAry(3))
    disp('Error at start or goal point!');
    path = zeros(0,3);
    num_expanded = 0;
    error('No Path Found!');
end

%% FIND THE GOAL
while any(size(reachable,1))&& ~visited(glAry(1), glAry(2), glAry(3))
    % Transfer node from sub to index.
    nodeInd = s2i(node);
    
    % Find the neighbors of new node in visited and define its cost vector.
    neighbors = nghbFind(map, node);
    costNghb = costCal(neighbors,node);
    
    % Check neighbors not in the visited.
    for i = 1:size(neighbors,1)
        % If the neighbors are visited skip this iteration.
    	if visited(neighbors(i,1), neighbors(i,2), neighbors(i,3))
			continue;
        end
        nghbInd = s2i(neighbors(i,:));
        % If the non-visited neighbors are not in the reachable, add them 
        % and update the cost.
        if ~rchFlag(neighbors(i, 1), neighbors(i, 2), neighbors(i, 3))%~ismember(neighbors(i,:),reachable,'rows')
            reachable = [reachable; neighbors(i, :)];
            rchFlag(neighbors(i, 1), neighbors(i, 2), neighbors(i, 3)) = 1;
            parent(nghbInd) = nodeInd;
            cost(nghbInd) = costNghb(i) + cost(nodeInd);
        % If they are already in the reachable, update the cost to be the minimum.
        elseif costNghb(i) + cost(nodeInd) < cost(nghbInd)
                cost(nghbInd) =  costNghb(i) + cost(nodeInd);
                parent(nghbInd) = nodeInd;
        end
    end
    
    % Find the minCost in reachable.
    if astar
    [~, minNum] = min(cost(s2i(reachable)) + astar * hrCost(reachable));
    else
    [~, minNum] = min(cost(s2i(reachable)));
    end
    
    % Extract minCost from reachable. Add node and minCost to visited and 
    % update current node.
    node = reachable(minNum,:);
    reachable(minNum, :) = [];
    rchFlag(node(1), node(2), node(3)) = 0;
    
    % Update visited.
    visited(node(1), node(2), node(3)) = 1;       
end

%% LOAD DATA
pathNum = 1;
pathPnt = s2i(glAry);
pathInd(1) = pathPnt;
pathSt = s2i(stAry);
    
while ~(pathPnt == pathSt)
    pathNum = pathNum + 1;
    pathPnt = parent(pathPnt);
    pathInd(pathNum) = pathPnt;
end
[pathX pathY pathZ]= i2s(pathInd);
pathAry = flipud([pathX(:),pathY(:),pathZ(:)]);
path = [ary2re(pathAry); goal];
path(1,:) = start;
num_expanded = sum(visited(:));
end