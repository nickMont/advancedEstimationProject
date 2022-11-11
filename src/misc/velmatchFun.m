function [uPurVM,uEvaVM,Ru2InflateP,Ru2InflateE] = velmatchFun(Spur,Seva,gameState)

targetLocation = gameState.Rtarget.x_target;
xPur=[gameState.xPur; gameState.xEva]; %full state according to pursuer
xTrue=xPur;
dt=gameState.dt;
umax=gameState.uMaxP;
upmax=umax;
uemax=umax;
utemp=Spur.utemp;
du=Spur.du;
uEvaEst=Spur.uEvaEstForVM;
uvec=Spur.uvec;
QtargetP=Spur.Jparams.Q_target;
QtargetE=Seva.Jparams.Q_target;
miscParams=gameState.miscParams;
vmtune=Spur.VMparams.vmtune;
scaleVec=Spur.VMparams.scalevec;

omega_hover=4.95;

xp=[xTrue(7:8);xTrue(10:11)];xe=[xTrue(19:20);xTrue(22:23)];
uP=vmRGVO_tune(xp,xe,umax,2,dt,zeros(2,1),vmtune);
uE=-scaleVec*uP;
xd = xPur(13:24); xd(9)=0;
uEvaVM = quadControllerACCONLY(xd, zeros(4,1), 3, [uE;0],0);
xd = xPur(1:12); xd(9)=0;
uPurVM = quadControllerACCONLY(xd, zeros(4,1), 3, [uP;0],0);

end