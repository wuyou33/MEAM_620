function h = hb(bphi,btheta,phi,psi,theta,x,y,z)
%HB
%    H = HB(BPHI,BTHETA,PHI,PSI,THETA,X,Y,Z)

%    This function was generated by the Symbolic Math Toolbox version 5.6.
%    28-Apr-2013 22:15:20

h = [x;y;z;bphi+phi;btheta+theta;psi];
