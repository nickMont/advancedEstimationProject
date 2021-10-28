function [uEst,Ru,uEvaTypeStack] = getUForMMKF(Spur,Seva,gameState,heurTypeStrucIJ,flagUseMeanInsteadOfLH2)
uEvaTemp=[];
Ru=[];

% unpacking information needed by gtFullScript
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

if nargin<=4
    flagUseMeanInsteadOfLH2=false;
end

Ru2InflateP=[];
Ru2InflateE=[];
if ~flagUseMeanInsteadOfLH2
    Ru2InflateP=zeros(4,4);
    Ru2InflateE=zeros(4,4);
end

if strcmp(heurTypeStrucIJ,'vmgt')
    FLAG_tryMotionPredictionInVM2=false;
    FLAG_tryVMGTbutBypassHeuristics=true;
    vmgtScript;
    uEvaTemp=uEvaVMGT;
    Ru=0.05*du*eye(4)+Ru2InflateE;
    uEvaTypeStack='nash-vmgt';
elseif strcmp(heurTypeStrucIJ,'gt-full')
    gtfullScript;
    uEvaTemp=uEvaGT;
    Ru=0.01*du*eye(4)+Ru2InflateE;
    uEvaTypeStack='nash-full';
elseif strcmp(heurTypeStrucIJ,'gt-pm')
    Qpur=Spur.Jparams.Q;
    Qeva=Seva.Jparams.Q;
    Rpur=Spur.Jparams.Rself;
    Reva=Seva.Jparams.Rself;
    loadPointMassControlParams;
    Ru=0.01*du*eye(4)+Ru2InflateE;
    uEvaTypeStack='nash-PM';
elseif strcmp(heurTypeStrucIJ,'vm')
    velmatchScript;
    uEvaTemp=uEvaVM;
    Ru=0.10*eye(4);
    uEvaTypeStack='vm';
elseif strcmp(heurTypeStrucIJ,'vmgt-heur')
    heurtype='both';
    vmgt_RA_HeurScript;
    uEvaTemp=uEvaVMGTH;
    Ru=0.10*eye(4)+Ru2InflateE;
    uEvaTypeStack='nash-vmgt-heur1';
elseif strcmp(heurTypeStrucIJ,'vmgt-heur2')
    heurtype='heur_only';
    vmgt_RA_HeurScript;
    uEvaTemp=uEvaVMGTH;
    Ru=0.10*eye(4)+Ru2InflateE;
    uEvaTypeStack{(iT-1)*nmod+ij,1}='nash-vmgt-heur2';
elseif strcmp(heurTypeStrucIJ,'other')
    uEvaTemp=omega_hover*ones(4,1);
    Ru=1*eye(4);
    uEvaTypeStack='hover';
end

uEst = uEvaTemp;

end

