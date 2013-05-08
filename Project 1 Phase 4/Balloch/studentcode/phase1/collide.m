function [C] = collide(map, points)
% COLLIDE Test whether points collide with an obstacle in an environment.
%   C = collide(map, points).  points is an M-by-3 matrix where each
%   row is an (x, y, z) point.  C in an M-by-1 logical vector; 
%   C(i) = 1 if M(i, :) touches an obstacle and is 0 otherwise.

global MapData

M=size(points,1);

C=zeros(M,1);

minx=MapData.minx;
miny=MapData.miny;
minz=MapData.minz;

discrete = [(points(:,1)-minx)/MapData.xy_res (points(:,2)-miny)/MapData.xy_res (points(:,3)-minz)/MapData.z_res];
discrete(discrete==0)= discrete(discrete==0)+eps;
discrete = ceil(discrete);

for i=1:M
    C(i) = map(discrete(i,1),discrete(i,2),discrete(i,3));
end
   


end
