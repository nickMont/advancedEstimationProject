function [Xdot] = epxv_dot_FL(t,X)

phi=X(1); theta=X(2); psi=X(3);
p=X(4); q=X(5); r=X(6); x=X(7); y=X(8); z=X(9);
vx=X(10); vy=X(11); vz=X(12); zeta=X(13); xi=X(14);

Ax=0;
Ay=0;
Az=0;
Ap=0;
Aq=0;
Ar=0;


[m,~, ~, ~, J, ~, ~, d] = get_dimensions();
Ix=J(1,1);
Iy=J(2,2);
Iz=J(3,3);

phidot=p+q*sin(phi)*tan(theta)+r*cos(phi)*tan(theta);
thetadot=q*cos(phi)-r*sin(phi);
psidot=p+q*sin(phi)*tan(theta)+r*cos(phi)*tan(theta);
pdot=(Iy-Iz)/Ix*q*r+Ap/Ix + d/Ix*ubar2;
qdot=(Iz-Ix)/Iy*p*r+Aq/Iy + d/Iy*ubar3;
rdot=(Ix-Iy)/Iz*p*q+Ar/Iz + 1/Iz*ubar4;
xdot=vx;
ydot=vy;
zdot=vz;
vxdot=Ax/m-1/m*(cos(phi)*cos(psi)*sin(theta)+sin(phi)*sin(psi))*zeta;
vydot=Ay/m-1/m*(cos(phi)*sin(theta)*sin(psi)-cos(psi)*sin(phi))*zeta;
vzdot=Az/m+g-1/m*cos(theta)*cos(phi)*zeta;
zetadot=xi;
xidot=ubar4;


Xdot=[phidot;thetadot;psidot;pdot;qdot;rdot;xdot;ydot;zdot;vxdot;vydot;vzdot;zetadot;xidot];


end

