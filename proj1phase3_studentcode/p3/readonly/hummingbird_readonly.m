function params = hummingbird_readonly()

m = 0.5;
g = 9.81;
I = [2.32e-3,0,0;0,2.32e-3,0;0,0,4e-3];
params.mass = m;
params.I = I;
params.invI = inv(I);
params.grav = g;



Ixx = I(1,1);
Iyy = I(2,2);
Izz = I(3,3);

t_attitude = 0.15; %rise time of the attitude controller
xi_attitude = 1; %damping ratio for attitude controller



params.maxangle = 40*pi/180; %you can specify the maximum commanded angle here
params.maxF = 2.5*m*g;
params.minF = 0.05*m*g;


%armlength is the distance from the center of the craft to the prop center
params.kforce = 6.11e-8; %in Newton/rpm^2
params.kmoment = 1.5e-9; %in Newton*meter/rpm^2
params.armlength = 0.175; %in meters

params.FM_omega2 = [params.kforce,params.kforce,params.kforce,params.kforce;...
    0,params.armlength*params.kforce,0,-params.armlength*params.kforce;...
    -params.armlength*params.kforce,0,params.armlength*params.kforce,0;...
    params.kmoment,-params.kmoment,params.kmoment,-params.kmoment];

params.omega2_FM = inv(params.FM_omega2);

params.maxomega = sqrt(params.maxF/(4*params.kforce));
params.minomega = sqrt(params.minF/(4*params.kforce));
