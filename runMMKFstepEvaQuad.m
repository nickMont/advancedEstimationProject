function [xhatE2,PhatE2] = runMMKFstepEvaQuad(SpurT,SevaT,gameStateT,miscParams,heurTypeStruc,muVeck)

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

Rk = 0.1*eye(12); %temp "fake" measurement covariance

uEvaEst=[0;0]; %for VM

Mij = miscParams.MMKFparams.Mij;
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

mu2=cell(nUe,1);
x2ent=cell(nUe,1);
numModels = nmod*miscParams.numTargets;
%     xhatE2 = xhatE;
%     PhatE2 = PhatE;
for ik=1:nUe
    x2ent{ik,1} = f_dynEvaQuad(xTrue(13:24),uEset{ik,1},dt,zeros(2,1));
    zMeas = x2ent{ik,1};
    mu = muVeck;
    
    xhatE = miscParams.MMKFparams.xhatE;
    PhatE = miscParams.MMKFparams.PhatE;
    lambdaTemp=-1*ones(nmod*miscParams.numTargets,1);
    for ij=1:nmod*miscParams.numTargets
        cc=zeros(numModels,1);
        for i2=1:numModels
            ccDum=0;
            for i3=1:numModels
                ccDum=ccDum+Mij(i3,ij)*mu(i3);
            end
            cc(i2)=ccDum;
        end
        muij=zeros(numModels,1);
        for j2=1:numModels
            muij(j2)=Mij(j2,ij)*1/cc(j2)*mu(j2);
        end
        uEvaMMF=uEset{ij};
        RuMMF=RuStack{ij};
        
        xBarijTemp=zeros(size(xhatE(:,ij)));
        PijTemp=zeros(size(PhatE(:,:,ij)));
        for j3=1:numModels
            xBarijTemp = xBarijTemp+xhatE(:,j3)*muij(j3);
        end
        for j4=1:numModels
            xDeltaTemp = xBarijTemp-xhatE(:,j4);
            PijTemp = PijTemp+muij(j4)*(PhatE(:,:,j4)+xDeltaTemp*xDeltaTemp');
        end
        
        Paug=PhatE(:,:,ij); Qk=RuMMF;
        if min(eig(Paug))>0 %check for impossible models
            % Propagate
            [xhatEp1,Pp1]=ukfPropagate(xBarijTemp,PijTemp,uEvaMMF,RuMMF,dt,'f_dynEvaQuad');
            
            % Measure
            [xhatE(:,ij),PhatE(:,:,ij),nu,Sk,~]=kfMeasure(xhatEp1,Pp1,zMeas,eye(12),Rk);
            Skt=triu(Sk); Skt=Skt'+Skt; Skt(1:13:end)=diag(Sk); %forces symmetry
            normpEval = mvnpdf(nu,zeros(12,1),Skt);
        else
            normpEval=1e-20;
        end
        if normpEval<=1e-8
            normpdf_eval=1e-8;
        end
        lambdaTemp(ij)=normpEval;
    end
    muTemp=muVeck;
    for ij=1:nmod*miscParams.numTargets
        muTemp(ij)=lambdaTemp(ij)*mu(ij)/dot(lambdaTemp,mu);
        if muTemp(ij)<1e-3
            muTemp(ij)=1e-3;
        end
    end
    mu2{ik,1}=muTemp/sum(muTemp);
    xhatE2(:,:,ik) = xhatE;
    PhatE2(:,:,:,ik) = PhatE;
end
end

