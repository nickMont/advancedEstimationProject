function [uPurVMGTH,uEvaVMGTH,Ru2InflateP,Ru2InflateE] = vmgt_RA_HeurFun(Spur,Seva,gameState,flagUseMeanInsteadOfLH2)

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

SevaTilde=Seva;
SpurTilde=Spur;
gameStateTilde=gameState;
gameStateTilde.Rtarget.x_target=targetLocation;
gameStateTilde.xPur=xPur(1:12);
gameStateTilde.xEva=xPur(13:24);
gameStateTilde.dt=dt;
gameStateTilde.kMax=1;
gameStateTilde.nu=2;
gameStateTilde.discType='overX';
gameStateTilde.uMaxP=umax;
SpurTilde.controlType='vmquad';

for ik=1:length(uvec)
    SpurTilde.uMat{ik}=upmax*uvec(ik);
end
SpurTilde.UseVelMatch=true;
SevaTilde.controlType='vmquad';
for ik=1:length(uvec)
    SevaTilde.uMat{ik}=upmax*uvec(ik);
end
SevaTilde.UseVelMatch=true;
if strcmp(heurtype,'both')
    gameStateTilde.Rtarget.useMotionPrediction=true;
    gameStateTilde.Rtarget.useNoHeuristics=false;
elseif strcmp(heurtype,'none')
    gameStateTilde.Rtarget.useMotionPrediction=false;
    gameStateTilde.Rtarget.useNoHeuristics=true;
elseif strcmp(heurtype,'pred_only')
    gameStateTilde.Rtarget.useMotionPrediction=true;
    gameStateTilde.Rtarget.useNoHeuristics=true;
elseif strcmp(heurtype,'heur_only')
    gameStateTilde.Rtarget.useMotionPrediction=false;
    gameStateTilde.Rtarget.useNoHeuristics=false;
else
    error('invalid argument to vmgt_RA_HeurScript');
end
[up,ue,flag,uPSampled,uESampled,Sminimax,Smisc,Smmkf2]=f_dyn2(SpurTilde,SevaTilde,gameStateTilde,zeros(4,1));
uPurVMGTH=uPSampled;
uEvaVMGTH=uESampled;

if exist('flagUseMeanInsteadOfLH2','var')
    if flagUseMeanInsteadOfLH2
        uPurVMGTH=Smmkf2.uValP;
        uEvaVMGTH=Smmkf2.uValE;
        Ru2InflateP=Smmkf2.RadditiveInflateP;
        Ru2InflateE=Smmkf2.RadditiveInflateE;
    end
end

end