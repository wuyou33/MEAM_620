function plot_path(map, path)
% PLOT_PATH Visualize a path through an environment
%   PLOT_PATH(map, path) creates a figure showing a path through the
%   environment.  path is an N-by-3 matrix where each row corresponds to the
%   (x, y, z) coordinates of one point along the path.

global MapData

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


% delete(1)
figure(1)
s=[size(map,1) size(map,2) size(map,3)];
[dx dy dz] = ind2sub(s, find(map));
% [x y z] = floor([dx*MapData.xy_res+MapData.minx dy*MapData.xy_res+MapData.miny dz*MapData.z_res+MapData.minz]);

d_map = [refx(dx)' refy(dy)' refz(dz)'];

if ~isempty(d_map)
    plot3(d_map(:,1), d_map(:,2), d_map(:,3), '.k');
end
hold on
if size(path,2)==3
    plot3(MapData.start(1),MapData.start(2),MapData.start(3),'bs')
    plot3(MapData.goal(1),MapData.goal(2),MapData.goal(3),'rs')

    plot3(path(:,1),path(:,2),path(:,3),'-go');
end
hold off
end