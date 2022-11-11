function [xP2,xE2] = runGameStepGivenUPur(SpurT,SevaT,gameStateT,miscParams,uPtrue,controlTypeEva,trueTargetIndex)
Spur = SpurT;
Seva = SevaT;
gameState = gameStateT;

dt = gameState.dt;
upmax = gameState.uMaxP;
uemax = upmax;
umax = upmax;
uvec = miscParams.uvec;
utemp = miscParams.utemp;
vmtune = miscParams.vmtune;
scaleVec = miscParams.scaleVec;
du = miscParams.du;

xE0 = gameState.xEva(7:9);

QtargetP = Spur.Jparams.Q_target;
QtargetE = Seva.Jparams.Q_target;
Qpur = Spur.Jparams.Q;
Rpur = Spur.Jparams.Rself;
Qeva = Seva.Jparams.Q;
Reva = Seva.Jparams.Rself;
FLAG_tryMotionPredictionInVM2=gameState.Rtarget.useMotionPrediction;
FLAG_tryVMGTbutBypassHeuristics=gameState.Rtarget.useNoHeuristics;

Rk = 0.1*eye(12); %temp "fake" measurement covariance

uEvaEst=[0;0]; %for VM
xPur = [gameState.xPur;gameState.xEva];
xTrue = xPur;

if nargin>=7
    targetLocation=miscParams.targetLocationVec{trueTargetIndex};
end

if strcmp(controlTypeEva,'vmgt')
    vmgtScript;
    uEvaTemp=uEvaVMGT;
    Ru=0.10*du*eye(4);
elseif strcmp(controlTypeEva,'gt-full')
    gtfullScript;
    uEvaTemp=uEvaGT;
    Ru=0.10*du*eye(4);
elseif strcmp(controlTypeEva,'gt-pm')
    loadPointMassControlParams;
    Ru=0.20*du*eye(4);
elseif strcmp(controlTypeEva,'vm')
    velmatchScript;
    uEvaTemp=uEvaVM;
    Ru=0.10*eye(4);
elseif strcmp(controlTypeEva,'vmgt-heur')
    heurtype='both';
    vmgt_RA_HeurScript;
    uEvaTemp=uEvaVMGTH;
    Ru=0.15*eye(4);
elseif strcmp(controlTypeEva,'vmgt-heur2')
    heurtype='heur_only';
    vmgt_RA_HeurScript;
    uEvaTemp=uEvaVMGTH;
    Ru=0.15*eye(4);
elseif strcmp(controlTypeEva,'other')
    uEvaTemp=omega_hover*ones(4,1);
    Ru=1*eye(4);
end
uEtrue = uEvaTemp;

xE2 = feval(Seva.fname,gameState.xEva,uEtrue,gameState.dt,zeros(6,1));

xP2 = feval(Spur.fname,gameState.xPur,uPtrue,gameState.dt,zeros(6,1));

end

