function rpy = newRot2RPY(R)
%written by Jonathan Balloch

% 
% http://planning.cs.uiuc.edu/node103.html
%

phi = atan2(R(3,2),R(3,3)); %gamma
theta = atan2(-R(3,1),sqrt( R(3,2)^2+R(3,3)^2 )); %beta
psi = atan2(R(2,1),R(1,1)); %alpha
rpy = [phi,theta,psi];
