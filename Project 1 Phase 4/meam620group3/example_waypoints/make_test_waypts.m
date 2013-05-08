%Make test waypoint lists for MEAM620 Proj 1 Phase 4

%all waypts are stored in matrices that are n x 3 where each row represents
%one waypoint, [x y z]

%Test waypoints 1. Just a line in the xy plane. 
l = 1:20;
waypts = [[-1:(1/10):1]', [-1:(1/10):1]', 1.2*ones(21,1)];
save('example_waypts_1','waypts')
figure;plot3(waypts(:,1),waypts(:,2),waypts(:,3),'b-o')
 hold on
%Test waypoints 2. 
waypts = [-1.2, -1.2, 1;
           0.2, -0.9, 1;
           0.2, -0.9, 2;
           0.5,  0.5, 2;
           0.5,  0.5, 1;
           1.7,  1.2, 1];
 save('example_waypts_2','waypts')
plot3(waypts(:,1),waypts(:,2),waypts(:,3),'r-o')

%Test waypoints 3. Diamond Helix thing.
waypts = [0, 0, 1;
          .25, -.35, 1.5;
          .5, 0, 2;
          .75, .35, 1.5;
          1, 0, 1];
save('example_waypts_3','waypts')
plot3(waypts(:,1),waypts(:,2),waypts(:,3),'g-o')
      


