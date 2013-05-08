function R = newRPY2Rot(phi,theta,psi)
%written by Jonathan Balloch

% BRW = [ cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), 
%     cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta), -cos(phi)*sin(theta)]
% 
% [                                 -cos(phi)*sin(psi),
%     cos(phi)*cos(psi),             sin(phi)]
% 
% [ cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi), 
%     sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi),
%     cos(phi)*cos(theta)]

R = [ cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta), -cos(phi)*sin(theta);...
    -cos(phi)*sin(psi),                                   cos(phi)*cos(psi),                                 sin(phi);...
    cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi), sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi), cos(phi)*cos(theta)];

%From Lavelle:

% R = [ cos(psi)*cos(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), cos(psi)*cos(phi)*sin(theta)+sin(phi)*sin(psi);...
%     cos(theta)*sin(psi), sin(phi)*sin(psi)*sin(theta) + cos(phi)*cos(psi),  cos(psi)*sin(phi) + cos(phi)*sin(theta)*sin(psi) ;...
%     sin(theta), cos(theta)*sin(phi), cos(phi)*cos(theta)];
