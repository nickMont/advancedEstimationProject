%note: this is written as a script rather than as a function to share
%access to workspace variables
xp=[xTrue(7:8);xTrue(10:11)];xe=[xTrue(19:20);xTrue(22:23)];
uP=vmRGVO_tune(xp,xe,umax,2,dt,zeros(2,1),vmtune);
uE=-scaleVec*uP;
xd = xPur(13:24); xd(9)=0;
uEvaVM = quadControllerACCONLY(xd, zeros(4,1), 3, [uE;0],0);
xd = xPur(1:12); xd(9)=0;
uPurVM = quadControllerACCONLY(xd, zeros(4,1), 3, [uP;0],0);