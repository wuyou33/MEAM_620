function [posEst, R, T] = pos_est_new(data, tagID, K1, BRC, tagPos)
% POS_EST estimates position.

% INPUT:
% data(structure) -- The data information as one cell in input "data".
% tagID -- The tagID matrix.
% K1 -- The inverse of camera intrinsic matrix.
% BRC -- Rotation matrix from the camera frame to robot frame.
% sft -- Shift matrix for tag computation.

% OUTPUT:
% posEst(1x6) -- The estimated position and orientation.
% R -- The rotation matrix.
% T -- The translation vector.

% Written by Qiong Wang for MEAM 620 at the University of Pennsylvania.
% April 9th, 2013

%% LOAD DATA.
tag = data.id;
tagNum = length(data.id);
p1 = K1 * [data.p1; ones(1,tagNum)];
p2 = K1 * [data.p2; ones(1,tagNum)];
p3 = K1 * [data.p3; ones(1,tagNum)];
p4 = K1 * [data.p4; ones(1,tagNum)];
pMat = [p1 p2 p3 p4];
A = zeros(8*tagNum,9);

%% HOMOGENEOUS LOOP SOLVING R and T.
% tagPos = zeros(2,tagNum);
% for j = 1:tagNum
%      for k = 1:4
%          % The camera coordinates.
%          x = pMat(1,(k-1)*tagNum+j);
%          y = pMat(2,(k-1)*tagNum+j);
% 
%          %Find the position of the tags.
%          [tagPos(:,j), ~] = get_tag(tag(j),tagID);
% 
%          % The world coordinates.
%          X = tagPos(1,j)+sft(1,k);
%          Y = tagPos(2,j)+sft(2,k);
% 
%          % The matrix.
%          A((8*(j-1)+2*k-1):(8*(j-1)+2*k),:) = ...
%                   [-X 0 x*X -Y 0 x*Y -1 0 x;
%                    0 -X y*X 0 -Y y*Y 0 -1 y];
%      end      
% end

for j = 1: tagNum
    
    % get the pos of tags under world frame
    P = tagPos{tagID == tag(j)}.pos;

    for k = 1:4
        % get the pos of tags under camera frame
        x = pMat(1,(k-1)*tagNum+j);
        y = pMat(2,(k-1)*tagNum+j);
        % The world coordinates.
        X = P(1, k);
        Y = P(2, k);
        % The matrix.
        A((8*(j-1)+2*k-1):(8*(j-1)+2*k),:) = ...
            [-X 0 x*X -Y 0 x*Y -1 0 x;
            0 -X y*X 0 -Y y*Y 0 -1 y];
    end
end



% SVD solving the value of lambdas.
[~, ~, V] = svd(A);
r = sign(V(end,end))*V(:,end);
M = reshape(r,3,3);
r1 = M(:,1)/norm(M(:,1));
r2 = M(:,2)/norm(M(:,1));
r3 = cross(r1,r2);
R = [r1 r2 r3];
T = M(:,3)/norm(M(:,1));
pos = R\([-0.04;0;-0.03]-T);
rpy = Rot2RPY(BRC*R);
posEst = [pos', rpy];

end