function [uEst,Ru,ummkfTypeStack] = getUForMMKF(Spur,Seva,gameState,heurTypeStrucIJ,flagUseMeanInsteadOfLH2,getPlayerType)
% approximates control actions for different controllers chosen by MMKF
% code is messy because of historical use of control scripts
uEvaTemp=[];
Ru=[];

% unpacking information needed by gtFullScript, etc
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
    uPurTemp=uPurVMGT;
    Ru=0.05*du*eye(4)+Ru2InflateE;
    ummkfTypeStack='nash-vmgt';
elseif strcmp(heurTypeStrucIJ,'gt-full')
    gtfullScript;
    uEvaTemp=uEvaGT;
    uPurTemp=uPurGT;
    Ru=0.01*du*eye(4)+Ru2InflateE;
    ummkfTypeStack='nash-full';
elseif strcmp(heurTypeStrucIJ,'gt-pm')
    Qpur=Spur.Jparams.Q;
    Qeva=Seva.Jparams.Q;
    Rpur=Spur.Jparams.Rself;
    Reva=Seva.Jparams.Rself;
    loadPointMassControlParams;
    Ru=0.01*du*eye(4)+Ru2InflateE;
    ummkfTypeStack='nash-PM';
elseif strcmp(heurTypeStrucIJ,'vm')
    velmatchScript;
    uEvaTemp=uEvaVM;
    Ru=0.10*eye(4);
    ummkfTypeStack='vm';
elseif strcmp(heurTypeStrucIJ,'vmgt-heur')
    heurtype='both';
    vmgt_RA_HeurScript;
    uEvaTemp=uEvaVMGTH;
    uPurTemp=uPurVMGTH;
    Ru=0.10*eye(4)+Ru2InflateE;
    ummkfTypeStack='nash-vmgt-heur1';
elseif strcmp(heurTypeStrucIJ,'vmgt-heur2')
    heurtype='heur_only';
    vmgt_RA_HeurScript;
    uEvaTemp=uEvaVMGTH;
    uPurTemp=uPurTempVMGTH;
    Ru=0.10*eye(4)+Ru2InflateE;
    ummkfTypeStack{(iT-1)*nmod+ij,1}='nash-vmgt-heur2';
elseif strcmp(heurTypeStrucIJ,'other')
    uEvaTemp=omega_hover*ones(4,1);
    uPurTemp=omega_hover*ones(4,1);
    Ru=1*eye(4);
    ummkfTypeStack='hover';
end


if nargin<=5
    getPlayerType='E';
end
if strcmpi(getPlayerType,'evader') || strcmpi(getPlayerType,'eva')
    getPlayerType='E';
end
if strcmpi(getPlayerType,'pursuer') || strcmpi(getPlayerType,'pur')
    getPlayerType='P';
end
if strcmpi(getPlayerType,'E')
    uEst = uEvaTemp;
elseif strcmpi(getPlayerType,'P')
    uEst=uPurTemp;
else
    error('Unrecognized player type in getUForMMKF');
end

end

