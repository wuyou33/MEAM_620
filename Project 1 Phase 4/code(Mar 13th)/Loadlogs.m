% Load High Level

load ViconData.mat 
load VelSave.mat 
load trpySave.mat 
load DesPosSave.mat 
load DesEulSave.mat 

b=find(trpySave>0);
[c d]=ind2sub(size(trpySave),b(end));
[e f]=ind2sub(size(trpySave),b(1));
trpyGood=trpySave(:,f:d);

b=find(DesPosSave>0);
[c d]=ind2sub(size(DesPosSave),b(end));
[e f]=ind2sub(size(DesPosSave),b(1));
DesPosGood=DesPosSave(:,f:d);

b=find(ViconData>0);
[c d]=ind2sub(size(ViconData),b(end));
[e f]=ind2sub(size(ViconData),b(1));
ViconGood=ViconData(:,f:d);

% b=find(DesEulSave>0);
% [c d]=ind2sub(size(DesEulSave),b(end));
% [e f]=ind2sub(size(DesEulSave),b(1));
% DesEulGood=DesEulSave(:,f:d);
% 
% b=find(DesVelSave>0);
% [c d]=ind2sub(size(DesVelSave),b(end));
% [e f]=ind2sub(size(DesVelSave),b(1));
% DesVelGood=DesVelSave(:,f:d);