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
colors=MapData.colors;


% delete(1)
figure(1)
 p = patch('Faces',MapData.faces,'Vertices',MapData.verts,'FaceColor','flat',...
     'FaceVertexCData',[colors;colors;colors;colors;colors;colors],'FaceAlpha',0.5);
 hold on

 if size(path,2)==3
    plot3(MapData.start(1),MapData.start(2),MapData.start(3),'bs')
    plot3(MapData.goal(1),MapData.goal(2),MapData.goal(3),'rs')

    plot3(path(:,1),path(:,2),path(:,3),'-g');
 end
 
xlabel('x'); ylabel('y'); zlabel('z');

hold off
end