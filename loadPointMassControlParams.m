%Loads simulation params for point mass model; written to use with quad
%simulation
%note: this is written as a script rather than as a function for historical
% reasons
xyd=[7,8,10,11]; %locations of xpos/ypos/xvel/yvel in 12-state
QpurPM = Qpur(xyd,xyd); RpurPM = 1/9/2*Rpur(1:2,1:2);
QevaPM = Qeva(xyd,xyd); RevaPM = 1/9/2*Reva(1:2,1:2);

%Preload GT params used in one or more GT solvers
gameState_pm.xPur=xTrue(xyd);
gameState_pm.xEva=xTrue(xyd+12);
gameState_pm.dt=dt;
gameState_pm.kMax=1;
gameState_pm.nu=2;
% uvelmatch=vmRGVO_max(xPur(1:4),xPur(5:8),upmax,2,dt,uEvaBestPerformanceEstimate);
for ik=1:length(utemp)
    Spur_pm.uMat{ik}=utemp(:,ik);
end
Spur_pm.Jname='J_pur';
Spur_pm.fname='f_dynPur';
Spur_pm.Jparams.Q=QpurPM;
Spur_pm.Jparams.Rself=RpurPM;
Spur_pm.Jparams.Ropp=zeros(2,2);
Seva_pm.uMat = Spur_pm.uMat;
% Spur_pm.uMat{length(utemp)+1}=uvelmatch;
Seva_pm.Jname='J_eva';
Seva_pm.fname='f_dynEva';
Seva_pm.Jparams.Q=QevaPM;
Seva_pm.Jparams.Rself=RevaPM;
Seva_pm.Jparams.Ropp=zeros(2,2);


[up,ue,flag]=f_dyn(Spur_pm,Seva_pm,gameState_pm,zeros(4,1));
if flag==0
    upp=randsample(1:length(Spur_pm.uMat),1,true,up);
    uPurPM=Spur_pm.uMat{upp}(:,1);
    uEvaTemp=zeros(gameState_pm.nu,gameState_pm.kMax);
    for ik=1:length(Seva_pm.uMat)
        uEvaTemp=uEvaTemp+ue(ik)*Seva_pm.uMat{ik}(:,1);
    end
else
    uPurPM=up;
    uEvaTemp=ue;
end

Ru2InflateP=zeros(4,4);
Ru2InflateE=zeros(4,4);

uEvaTemp = quadControllerACCONLY(xTrue(13:24), zeros(4,1), 3, [uEvaTemp;0],0);
uPurTemp = quadControllerACCONLY(xTrue(13:24), zeros(4,1), 3, [uPurPM;0],0);








