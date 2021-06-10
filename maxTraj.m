function [uPur,Jset] = maxTraj(SpurT,SevaT,gameStateT,miscParams,uPset,heurTypeStruc,muVeck)

nUset = length(uPset);

xP2=cell(nUset,1);
nUe = miscParams.numTargets*length(heurTypeStruc);
xE2=cell(nUe,1);

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
        elseif strcmp(heurTypeStruc{ij},'gt-full')
            gtfullScript;
            uEvaTemp=uEvaGT;
        elseif strcmp(heurTypeStruc{ij},'gt-pm')
            loadPointMassControlParams;
        elseif strcmp(heurTypeStruc{ij},'vm')
            velmatchScript;
            uEvaTemp=uEvaVM;
        elseif strcmp(heurTypeStruc{ij},'vmgt-heur')
            heurtype='both';
            vmgt_RA_HeurScript;
            uEvaTemp=uEvaVMGTH;
        elseif strcmp(heurTypeStruc{ij},'vmgt-heur2')
            heurtype='heur_only';
            vmgt_RA_HeurScript;
            uEvaTemp=uEvaVMGTH;
        elseif strcmp(heurTypeStruc{ij},'other')
            uEvaTemp=omega_hover*ones(4,1);
        end
        uEset{nn}=uEvaTemp;
    end
end

%propagate each evader type
nn=0;
for ij = 1:nUe
%     uu=uEset{ij}
    xE2{ij} = feval(Seva.fname,gameState.xEva,uEset{ij},gameState.dt,zeros(6,1));
end

%propagate each input control
for ij = 1:nUset
    dx = uPset{ij};
    x2=xTrue(1:12); x2(7:8)=x2(7:8)+dx;
    uPT = quadController(xTrue(1:12),zeros(4,1),zeros(3,1),x2,1,zeros(12,1));
    xP2{ij} = feval(Spur.fname,gameState.xPur,uPT,gameState.dt,zeros(6,1));
end

Jtraj = zeros(nUset,1);

indexInCost = allcomb(1:nUe,1:nUe);
nonRepList=(indexInCost(:,1)~=indexInCost(:,2));
indexInCost = indexInCost(nonRepList==1,:);
for ip = 1:nUset
    gameState.xPur = xP2{ip};
    uEset={};
    nn=0;
    for iT=1:miscParams.numTargets
        targetLocation=miscParams.targetLocationVec{iT};
        for ij=1:nmod
            gameState.xEva=xE2{ij};
            xPur = [xP2{ip};xE2{ij}];
            xTrue = xPur;
            nn=nn+1;
            uEvaTemp=[];
            if strcmp(heurTypeStruc{ij},'vmgt')
                vmgtScript;
                uEvaTemp=uEvaVMGT;
            elseif strcmp(heurTypeStruc{ij},'gt-full')
                gtfullScript;
                uEvaTemp=uEvaGT;
            elseif strcmp(heurTypeStruc{ij},'gt-pm')
                loadPointMassControlParams;
            elseif strcmp(heurTypeStruc{ij},'vm')
                velmatchScript;
                uEvaTemp=uEvaVM;
            elseif strcmp(heurTypeStruc{ij},'vmgt-heur')
                heurtype='both';
                vmgt_RA_HeurScript;
                uEvaTemp=uEvaVMGTH;
            elseif strcmp(heurTypeStruc{ij},'vmgt-heur2')
                heurtype='heur_only';
                vmgt_RA_HeurScript;
                uEvaTemp=uEvaVMGTH;
            elseif strcmp(heurTypeStruc{ij},'other')
                uEvaTemp=omega_hover*ones(4,1);
            end
            %         uEvaTempStack{(iT-1)*nmod+ij,1}=uEvaTemp;
            %         RuStack{(iT-1)*nmod+ij,1}=Ru;
            uEset{nn}=uEvaTemp;
        end
    end
    xE3 = cell(nUe,1);
    for ie = 1:nUe
        x3temp = feval(Seva.fname,gameState.xEva,uEset{nn},gameState.dt,zeros(6,1));
        xE3{ie} = x3temp(7:9);
    end
    Jt=0;
    for iJ = 1:length(indexInCost)
        i1 = indexInCost(iJ,1);
        i2 = indexInCost(iJ,2);
        Jt=Jt+muVeck(i1)*muVeck(i2)*norm(cross(xE3{i1},xE3{i2}));
    end
    Jtraj(ip) = Jt;
end

Jset = Jtraj;
[minv,mindex]=min(Jtraj);
uPur = uPset{mindex};

end

