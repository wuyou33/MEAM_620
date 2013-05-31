function params = nanoplus()

m = .176; %kg
g = 9.81;
I = [0.00025,0,2.55e-6;0,0.000232,0;2.55e-6,0,0.0003738]; %see mathematica calculation
params.mass = m;
params.I = I;
params.invI = inv(I);
params.grav = g;


Ixx = I(1,1);
Iyy = I(2,2);
Izz = I(3,3);


params.maxangle = 40*pi/180; %you can specify the maximum commanded angle here
params.maxF = 2.5*m*g;
params.minF = 0.05*m*g;


%F = param.kforce * omega^2
%M = params.kmoment * omega^2
%armlength is the distance from the center of the craft to the prop center
% params.kforce = 6.11e-8; %in Newton/rpm^2 
% params.kmoment = 1.5e-9; %in Newton*meter/rpm^2
% params.armlength = 0.0849; %in meters
% 
% params.FM_omega2 = [params.kforce,params.kforce,params.kforce,params.kforce;...
%     0,params.armlength*params.kforce,0,-params.armlength*params.kforce;...
%     -params.armlength*params.kforce,0,params.armlength*params.kforce,0;...
%     params.kmoment,-params.kmoment,params.kmoment,-params.kmoment];
% 
% params.omega2_FM = inv(params.FM_omega2);
% 
% params.maxomega = sqrt(params.maxF/(4*params.kforce));
% params.minomega = sqrt(params.minF/(4*params.kforce));
