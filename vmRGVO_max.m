function u = vmRGVO_max(xP,xE,umaxP,nX)
% http://cimlab.mie.utoronto.ca/wp-content/uploads/2018/06/Kunwar-2006-Rendezvous-guidance-trajectory-planning-for-robotic-dynamic-obstacle-avoidance-and-interception.pdf
% xP/xE is full state of position/velocity
% returns nan if no control is possible
rP=xP(1:nX);
rE=xE(1:nX);
r=rE-rP;

umaxDecel=sqrt(norm(r)*2*umaxP);

%Find a that satisfies:
% cross(r,rdot)=0; dot(r,rdot)<0;

u=-umaxDecel*r/norm(r);


end

