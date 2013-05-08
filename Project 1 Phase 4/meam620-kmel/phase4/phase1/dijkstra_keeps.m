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

global MapData;

MapData.goal=goal;
MapData.start=start;


minx=MapData.minx;
miny=MapData.miny;
minz=MapData.minz;
maxx=MapData.maxx;
maxy=MapData.maxy;
maxz=MapData.maxz;
xy_res=MapData.xy_res;
z_res=MapData.z_res;

refx=linspace(minx,maxx,2*size(map,1)+1);
refx=refx(2:2:end);
refy=linspace(miny,maxy,2*size(map,2)+1);
refy=refy(2:2:end);
refz=linspace(minz,maxz,2*size(map,3)+1);
refz=refz(2:2:end);

if nargin < 4
    astar = true;
elseif sum(strcmp(astar,{'T','TRUE','True','true'}))
    astar = true;
else
    astar = false;
end

if astar==0
    h=0;
else
    h=1; %h may not be greater than 1
end

% visited in discrete. bounded unoccupied volume
V=zeros(size(map(map==0))); 
% reachable neighbors in discrete. bounded 2/3*unoccupied volume
R=zeros( 1, ceil(2*size(map(map==0),1)/3) ); 
% Maximum path = 1/9 of unoccupied volume
pathfind=zeros(ceil(size(map(map==0),1)/9),1);

% path = zeros(size(map(map==0),1),3);
num_expanded = 0;
stop=1;

%initialize parent
parent=-ones(size(map));

% vector to add to current to find neighbors
% idx version
% neighbor=[-1,1,-size(map,1)+1,-size(map,1),-size(map,1)-1,size(map,1)+1,size(map,1),size(map,1)-1];
% neighbor= [neighbor,size(map,1)*size(map,2),neighbor+size(map,1)*size(map,2),...
%     -size(map,1)*size(map,2),neighbor-size(map,1)*size(map,2)];

%sub version
neighbor = [-1 1 -1  0  1 -1 0 1 -1  0  1 -1  0  1 -1  0  1 -1 0 1 -1  0  1 -1 0 1;
             0 0 -1 -1 -1  1 1 1  0  0  0 -1 -1 -1  1  1  1  0 0 0 -1 -1 -1  1 1 1;
             0 0  0  0  0  0 0 0 -1 -1 -1 -1 -1 -1 -1 -1 -1  1 1 1  1  1  1  1 1 1]';

%neighbor costs, ordered
middlecost= [xy_res xy_res xy_res*1.4142 xy_res xy_res*1.4142 xy_res*1.4142 xy_res xy_res*1.4142];
cross=sqrt(z_res^2+xy_res^2);
corners=sqrt((xy_res*1.4142)^2+z_res^2);
topbottom= [ cross z_res cross corners cross corners corners cross corners];
neighcost= [middlecost topbottom topbottom];


%discretize start and goal and give initial values for R
disc_start = [(start(1)-minx)/xy_res (start(2)-miny)/xy_res (start(3)-minz)/z_res];
disc_start(disc_start==0)= disc_start(disc_start==0)+eps;
disc_start = ceil(disc_start);
startidx=sub2ind( size(map),disc_start(1),disc_start(2),disc_start(3) );

disc_goal = [(goal(1)-minx)/xy_res (goal(2)-miny)/xy_res (goal(3)-minz)/z_res];
disc_goal(disc_goal==0)= disc_goal(disc_goal==0)+eps;
disc_goal = ceil(disc_goal);
goalidx=sub2ind( size(map),disc_goal(1),disc_goal(2),disc_goal(3) );

%cost will always be in actual cost, which is discrete_cost*rez
cost=Inf(size(map));
if h~=0
    [a b c]=ind2sub(size(cost),find(cost));
    cost(:) = sqrt( (-refx(a')'+refx(disc_goal(1))).^2 +  (-refy(b')'+refy(disc_goal(2))).^2 + (-refz(c')'+refz(disc_goal(3))).^2 );
    cost(map==1)=Inf;
end
cost(startidx)=0;
if h~=0
    euclid=cost;
else
    euclid=0;
end
parent(startidx)=0; %zero is sufficiently "null" in Matlab

R(1)=startidx;


% Progress Plotting
%
% pause(0.001);
% plot_path(map,pathfind);
% hold on
%
%


while ( ~isempty(R(R~=0)) && isempty(V(V==goalidx)) )
    %find next
    [p_cost Ridx]=min(cost(R(R~=0)));
    V(num_expanded+1)=R(Ridx);

    
    % real coordinate, update path
    [dx dy dz]=ind2sub(size(map),R(Ridx));
    
%     path( num_expanded+1,:) = [refx(dx) refy(dy) refz(dz)];
    
    %find new neighbors, avoid edges and collisions, visited
    neighbors=bsxfun(@plus,neighbor,[dx dy dz] );
    
    %keep finds the neighbors that are inside the map boundaries
    keep=( prod(single(neighbors>0),2)>0 & prod(single( bsxfun(@le,neighbors,[size(map,1) size(map,2) size(map,3)])),2)>0);
    neighcosts=neighcost(keep);    
    neighbors=neighbors(keep,:);
    neighbors=sub2ind( size(map),neighbors(:,1),neighbors(:,2),neighbors(:,3))';
    
    %keep2 removes neighbors that collide or are in V
    keep2=( map(neighbors) == 0 & ~sum(bsxfun(@eq,neighbors,V(V~=0)),1) );
    neighcosts= neighcosts(keep2);
    neighbors= neighbors(keep2);

    % h=0 when astar false, so cost is just new_neighcosts+p_cost
    neighcosts=neighcosts+(p_cost);
    if h~=0
        neighcosts=neighcosts-euclid(R(Ridx)) + h*cost(neighbors);
    end
    
    % removes neighbors already in R
%     keep3= (~sum( bsxfun(@eq,neighbors,R(R~=0)') & bsxfun(@gt,neighcosts,cost(R(R~=0))') ,1));
    keep3= ~sum( bsxfun(@eq,neighbors,R(R~=0)'),1);% & bsxfun(@gt,neighcosts,cost(R(R~=0))') ,1));
 
    new_neighcosts=neighcosts(keep3);
    new_neighbors=neighbors(keep3);
    
%     new_neighcosts=neighcosts(~sum(bsxfun(@eq,neighbors,[R';V']),1));
%     new_neighbors=neighbors(~sum(bsxfun(@eq,neighbors,[R';V']),1));
    
    
    % Update cost and parent
    parent(new_neighbors) = R(Ridx);
    
    cost(new_neighbors) = new_neighcosts;
    if (map(new_neighbors) == 1)
        cost(new_neighbors(map(new_neighbors)==1))=Inf;
    end
    % clean up R and V
    %preallocated
    R=[R(R~=R(Ridx)) 0];
    R( (size(R(R~=0),2)+1):(size(R(R~=0),2)+size(new_neighbors,2)) ) = new_neighbors;
%     R(~sum(bsxfun(@eq,R,V'),1))=0; %%%%%%%%%

    
    %non-preallocated
%     R = [R new_neighbors];
%     V=[V R(Ridx)];
%     R=unique( [ R(R~=R(Ridx))  R(~sum(bsxfun(@eq,R,V'),1)) ] ); %%%%%%%%%
    
    num_expanded=num_expanded+1;
    
    
    
    
    % Progress Plotting 
    %
%     pause(0.001);
%     plot3(refx(dx),refy(dy),refz(dz),'o','MarkerFaceColor',[.49 1 0]);
    %
    %
end

% if ( ~isempty(R(R~=0)))
%     return
% end

% Progress Plotting
% hold off
% close 1
%

if ~isempty(V)
%   path = zeros(size(map(map==0),1),3);
%   path=path(prod(single(bsxfun(@ne,path,[0 0 0])),2)>0,:);
    pathfind(end)=goalidx;
    count=0;
    
    %in pathfind you only need to search the neighbors!!! rewrite
    while isempty(pathfind(pathfind==startidx))
        count=count+1;
%         if %count==80*stop
%             stop=stop+1;
%         end
        pathfind(end-count) = parent(pathfind(end-count+1));

    end
    
    pathfind=pathfind(pathfind~=0);
    [path(:,1) path(:,2) path(:,3)]=ind2sub(size(map),pathfind);
    path(:,1)= refx(path(:,1));
    path(:,2)= refy(path(:,2));
    path(:,3)= refz(path(:,3));
    path(1,:)=start;
    path(end,:)=goal;
%     plot_path(map,path);
end
