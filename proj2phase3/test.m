% vicon = zeros(6,lengDat);
% for i = 1: lengDat
%     t(i) = sensorLog{i}.t;
%     vicon(1:3,i) = qdLog{i}{1}.euler;
%     vicon(4:6,i) = qdLog{i}{1}.pos;
% end
% plot(t,vicon(4,:),'r','LineWidth',1.5);
i = 1;
t = zeros(2,lengDat);
t0 = [qdLog{i}{1}.vtime; qdTimeLog{i};];
for i = 2:lengDat
    t(:,i) = [qdLog{i}{1}.vtime; qdTimeLog{i};]-t0;
end
plot(1:lengDat, t)