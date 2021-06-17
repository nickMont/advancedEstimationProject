function [uEset] = getControlSetForMultiControllerQuadEva(SpurT,SevaT,gameStateT,miscParams,heurTypeStruc)
nUe = miscParams.numTargets*length(heurTypeStruc);

uEset=cell(nUe,1);

Spur = SpurT;
Seva = SevaT;
gameState = gameStateT;

nmod=length(heurTypeStruc);
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

uEvaEst=[0;0]; %for VM

xPur = [gameState.xPur;gameState.xEva];
xTrue = xPur;

nn=0;
for iT=1:miscParams.numTargets
    targetLocation=miscParams.targetLocationVec{iT};
    for ij=1:nmod
        nn=nn+1;
        uEvaTemp=[];
        %         tt = heurTypeStruc{ij}
        if strcmp(heurTypeStruc{ij},'vmgt')
            vmgtScript;
            uEvaTemp=uEvaVMGT;
            Ru=0.10*du*eye(4);
        elseif strcmp(heurTypeStruc{ij},'gt-full')
            gtfullScript;
            uEvaTemp=uEvaGT;
            Ru=0.10*du*eye(4);
        elseif strcmp(heurTypeStruc{ij},'gt-pm')
            loadPointMassControlParams;
            Ru=0.20*du*eye(4);
        elseif strcmp(heurTypeStruc{ij},'vm')
            velmatchScript;
            uEvaTemp=uEvaVM;
            Ru=0.10*eye(4);
        elseif strcmp(heurTypeStruc{ij},'vmgt-heur')
            heurtype='both';
            vmgt_RA_HeurScript;
            uEvaTemp=uEvaVMGTH;
            Ru=0.15*eye(4);
        elseif strcmp(heurTypeStruc{ij},'vmgt-heur2')
            heurtype='heur_only';
            vmgt_RA_HeurScript;
            uEvaTemp=uEvaVMGTH;
            Ru=0.15*eye(4);
        elseif strcmp(heurTypeStruc{ij},'other')
            uEvaTemp=omega_hover*ones(4,1);
            Ru=1*eye(4);
        end
        uEset{nn}=uEvaTemp;
        RuStack{nn}=Ru;
    end
end
end

