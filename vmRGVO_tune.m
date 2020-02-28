function u = vmRGVO_tune(xP,xE,umaxP,nX,dt,uEvaEst,decelParam)
% http://cimlab.mie.utoronto.ca/wp-content/uploads/2018/06/Kunwar-2006-Rendezvous-guidance-trajectory-planning-for-robotic-dynamic-obstacle-avoidance-and-interception.pdf
% xP/xE is full state of position/velocity
% returns nan if no control is possible
rP=xP(1:nX);
rE=xE(1:nX);
r=rE-rP;
dv=(xE(nX+1:end)-xP(nX+1:end));
vPara=dot(r,dv);

rUnit=r/norm(r);

if nargin<=4
    dt=1;
    fprintf('Check narguments')
end
if nargin<=5
    uEvaEst=[0;0];
end

umaxDecel=decelParam*sqrt(norm(r)*2*umaxP);

%Find a that satisfies:
% cross(r,rdot)=0; dot(r,rdot)<0;

uMag=umaxDecel;
if abs(norm(r))<abs(vPara*dt+1/2*umaxDecel*dt^2)
    uE2=f_dynEva(xE,uEvaEst,dt,zeros(2,1));
    fun=@(u) norm(uE2-f_dynPur(xP,u,dt,zeros(2,1)));
    u=fminsearch(fun,[0;0]);
else
    if abs(uMag)>umaxP
        uMag=sign(uMag)*umaxP;
    end
    u=uMag*rUnit;
end


end

