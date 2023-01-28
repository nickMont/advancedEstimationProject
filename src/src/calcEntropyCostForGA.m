function entropyVec = calcEntropyCostForGA(SpurT,SevaT,gameStateT,miscParams,uPset,heurTypeStruc,muVeck,tSimMMKF)
Spur=SpurT;
Seva=SevaT;
gameState=gameStateT;

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

xPur = [gameState.xPur;gameState.xEva];
xTrue = xPur;
nn=0;
RuStack=cell(nUe,1);


% Calc mu2 IF using entropy
Mij = miscParams.MMKFparams.Mij;

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


uEvaTempStack=cell(nmod,1);
% Create single loop over control models to make cleaner code
% NOTE: column 1 should be targets, 2 should be controller types. This
% aligns with original inputs of Target (outerloop) X Heur (innerloop)
controlXtargetCombs = allcomb(1:miscParams.numTargets,1:length(heurTypeStruc));
nU2 = length(controlXtargetCombs);

mu3=cell(nUset,1);
ip=1;
muBlock = zeros(nU2,nU2);
for ikUxT=1:nU2
    xPur = [xP2{ip};x2ent{ikUxT}];
    mu = mu2{ikUxT,1};

    ikH = controlXtargetCombs(ikUxT,2);
    ikT = controlXtargetCombs(ikUxT,1);
    targetLocation=miscParams.targetLocationVec{ikT};

    xEsim = x2ent{ikUxT,1};
    xPsim = xP2{ip};

    for iT=1:tSimMMKF

        xPur=[xPsim;xEsim];

        for inn2 = 1:nU2
            ikFakeH = controlXtargetCombs(inn2,2);
            ikFakeT = controlXtargetCombs(inn2,1);
            targetLocation=miscParams.targetLocationVec{ikFakeT};
            if strcmp(heurTypeStruc{ikFakeH},'vmgt')
                vmgtScript;
                uEvaTemp=uEvaVMGT;
            elseif strcmp(heurTypeStruc{ikFakeH},'gt-full')
                gtfullScript;
                uEvaTemp=uEvaGT;
            elseif strcmp(heurTypeStruc{ikFakeH},'gt-pm')
                loadPointMassControlParams;
            elseif strcmp(heurTypeStruc{ikFakeH},'vm')
                velmatchScript;
                uEvaTemp=uEvaVM;
            elseif strcmp(heurTypeStruc{ikFakeH},'vmgt-heur')
                heurtype='both';
                vmgt_RA_HeurScript;
                uEvaTemp=uEvaVMGTH;
            elseif strcmp(heurTypeStruc{ikFakeH},'vmgt-heur2')
                heurtype='heur_only';
                vmgt_RA_HeurScript;
                uEvaTemp=uEvaVMGTH;
            elseif strcmp(heurTypeStruc{ikFakeH},'other')
                uEvaTemp=omega_hover*ones(4,1);
            end
            uEvaTempStack{inn2,1}=uEvaTemp;
        end
        uEvaTrueTemp = uEvaTempStack{ikUxT,1}; %match MMKF control to correct control for iteration

        %sim lop here
        %xE
        xEsim = f_dynEvaQuad(xEsim,uEvaTrueTemp,dt,zeros(2,1));
        zMeas = xEsim;

        %xP
        dx = uPset{ip};
        dx2 = xPsim; dx2(7:8)=dx2(7:8)+dx;
        uPT = quadController(xPsim,zeros(4,1),zeros(3,1),dx2,1,zeros(12,1));
        xPsim = f_dynPurQuad(xPsim,uPT,dt,zeros(2,1));

        %run MMKF
        xhatE = xhatE2(:,:,ikUxT);
        PhatE = PhatE2(:,:,:,ikUxT);
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
        muTemp=mu;
        for ij=1:nmod*miscParams.numTargets
            muTemp(ij)=lambdaTemp(ij)*mu(ij)/dot(lambdaTemp,mu);
            if muTemp(ij)<1e-3
                muTemp(ij)=1e-3;
            end
        end
        muTemp = muTemp/sum(muTemp);
        muBlock(:,ikUxT) = muTemp;
        mu=muTemp;
    end
end
mu3{ip,1}=muBlock;

muTilde = mu3{ip,1};
Ht=zeros(nU2,1);
%         mu0 = muVeck;
%         muTilde = sum(muTilde.*mu0,2);
for ie=1:nU2
    Ht(ie)=calcEntropy(muTilde(:,ie));
end

entropyVec=Ht;


end