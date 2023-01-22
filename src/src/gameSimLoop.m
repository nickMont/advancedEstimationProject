function [thist,xStore,uPhist,uEhist,JJp,JJe] = gameSimLoop(Spur,Seva,gameState,miscParams,flags)

% Take normal filling of gameState/Spur/Seva/etc and add: 
%
%     miscParams.numTargets = numTargets;
%     miscParams.targetLocationVec = targetLocationVec;
%     miscParams.uvec = uvec;
%     miscParams.utemp = utemp;
%     miscParams.vmtune = vmtune;
%     miscParams.scaleVec = scaleVec;
%     miscParams.du = du;
%     miscParams.MMKFparams.xhatE = xhatE;
%     miscParams.MMKFparams.PhatE = PhatE;
%     miscParams.MMKFparams.Mij = Mij;
%     miscParams.dt=dt;
%     miscParams.tMax=tmax;
%     miscParams.umax=umax;
%     flags.useGameTheoreticController = FLAG_useGameTheoreticController;
%     flags.usePurVelMatchController = FLAG_usePureVelMatchController;
%     miscParams.uEvaTrueTargetIndex = evaTrueTargetIndex;
%     miscParams.Rk=Rk;
%     [thist,xStore,uPhist,uEhist,JJp,JJe] = gameSimLoop(Spur,Seva,gameState,miscParams,flags);


uPhist=[];
uEhist=[];
thist=[];
xStore=[];

JJp=0;
JJe=0;

gameStateT=gameState;
SpurT=Spur;
SevaT=Seva;
dt=gameState.dt;

numRefinements=0;

numTargets=miscParams.numTargets;
targetLocationVec=miscParams.targetLocationVec;

xTrue=[gameState.xPur;gameState.xEva];
xPur=xTrue;
xEva=xTrue;
umax=miscParams.umax;

FLAG_useGameTheoreticController = flags.useGameTheoreticController;
FLAG_usePureVelMatchController = flags.usePurVelMatchController;
evaTrueTargetIndex=miscParams.uEvaTrueTargetIndex;

Rk=miscParams.Rk;

n=0;
for t=0:dt:miscParams.tMax
    tLoop = t
    n=n+1;
    
%     Spur.Jname='J_purQuadTarget';
%     Seva.Jname='J_evaQuadTarget';
%     Spur.Jparams.Q=Qpur;
%     Spur.Jparams.Rself=Rpur;
%     Spur.Jparams.Ropp=zeros(4,4);
%     Spur.uLmax=uLmax;
%     Seva.Jparams.Q=Qeva;
%     Seva.Jparams.Rself=Reva;
%     Seva.Jparams.Ropp=zeros(4,4);
%     Seva.uLmax=uLmax;
    
    % Robust estimation params
%     miscParams.Qk=0.001*eye(24);
%     miscParams.useUT=false;
    
    
    uEvaEst=zeros(2,1);
    
    % Guessing pursuer
%     gameStateT.xPur=xPur(1:12);
%     gameStateT.xEva=xPur(13:24);
%     gameState.dt=dt;
%     gameState.kMax=1;
%     gameState.nu=2;
%     gameState.discType='overX';
%     gameState.uMaxP=upmax;
%     gameState.uEvaEstForVM=uEvaEst;
%     Spur.uMat={}; Seva.uMat={};
%     if strcmp(Spur.controlType,'vmquad')
%         for ik=1:length(uvec)
%             Spur.uMat{ik}=upmax*uvec(ik);
%         end
%     end
%     if strcmp(Spur.controlType,'gt_overx')
%         for ik=1:length(utemp)
%             Spur.uMat{ik}=upmax*utemp(:,ik);
%         end
%     end
%     Spur.Jname='J_purQuadTarget';
%     Spur.fname='f_dynPurQuad';
%     Spur.UseVelMatch=true;
%     if strcmp(Seva.controlType,'vmquad')
%         for ik=1:length(uvec)
%             Seva.uMat{ik}=uemax*uvec(ik);
%         end
%     end
%     if strcmp(Seva.controlType,'gt_overx')
%         for ik=1:length(utemp)
%             Seva.uMat{ik}=uemax*utemp(:,ik);
%         end
%     end
%     Spur.Jparams.Q_target=QtargetP;
%     Seva.Jname='J_evaQuadTarget';
%     Seva.fname='f_dynEvaQuad';
%     Seva.Jparams.Q_target=QtargetE;
%     
%     gameState.uEvaEstForVM=uEvaEst;
%     gameState.Rtarget.Q_target=QtargetP;
%     gameState.Rtarget.useMotionPrediction=FLAG_tryMotionPredictionInVM2;
%     gameState.Rtarget.useNoHeuristics=FLAG_tryVMGTbutBypassHeuristics;
%     Seva.UseVelMatch=true;
    
    %     if t>3
    %         fprintf('manual swap at line 150')
    %         evaTrueTargetIndex=2;
    %     end
    
    gameStateT.xPur=xPur(1:12);
    gameStateT.xEva=xPur(13:24);
    
    for iT=1:numTargets
        targetLocation=targetLocationVec{iT};
        if FLAG_useGameTheoreticController
            for iR=1:numRefinements+1
                gameState.Rtarget.x_target=targetLocation;
                Seva.Jparams.x_target=targetLocation;
                Spur.Jparams.x_target=targetLocation;
                % Propagate to next time step
                [up,ue,flag,uPSampled,uESampled,Sminimax,Smisc]=f_dyn2(SpurT,SevaT,gameStateT,zeros(4,1),miscParams);
                uPurTrueStr{iT}=uPSampled;
                uEvaTrueStr{iT}=uESampled;
                uEvaNash=uESampled;
                dewxv=f_dynEvaQuad(xTrue(13:24),uEvaNash,dt,zeros(2,1))-gameState.xEva;
                dx=dewxv(7:8); dx=unit_vector(dx);
                uEvaEst=umax*dx;
            end
        elseif FLAG_usePureVelMatchController
            %velmatchScript
            xp=[xTrue(7:8);xTrue(10:11)];xe=[xTrue(19:20);xTrue(22:23)];
            uP=vmRGVO_tune(xp,xe,umax,2,dt,zeros(2,1),vmtune);
            uE=-scaleVec*uP;
            xd = xPur(13:24); xd(9)=0;
            uEvaTrueStr{iT} = quadControllerACCONLY(xd, zeros(4,1), 3, [uE;0],0);
            xd = xPur(1:12); xd(9)=0;
            uPurTrueStr{iT} = quadControllerACCONLY(xd, zeros(4,1), 3, [uP;0],0);
        else
            error('No controller specified')
        end
    end
    
    
    %Determine which target's control should be applied
    uPurTrue=uPurTrueStr{1};
    uEvaTrue=uEvaTrueStr{evaTrueTargetIndex};

    
%     if max(n==indexToRunInfo)==1
%         uInfoMat={};
%         uInfoMat{1}=[0;0];
%         for ii=1:4
%             uInfoMat{1+ii}=upmax*[cos(ii*pi/2);sin(ii*pi/2)];
%         end
%         miscParams.numTargets = numTargets;
%         miscParams.targetLocationVec = targetLocationVec;
%         miscParams.uvec = uvec;
%         miscParams.utemp = utemp;
%         miscParams.vmtune = vmtune;
%         miscParams.scaleVec = scaleVec;
%         miscParams.du = du;
%         miscParams.MMKFparams.xhatE = xhatE;
%         miscParams.MMKFparams.PhatE = PhatE;
%         miscParams.MMKFparams.Mij = Mij;
%         SpurT=Spur;
%         SpurT.uMat=purInfoControlSet;
%         uPur_dx = maxTraj(SpurT,Seva,gameState,miscParams,uInfoMat,heurTypeStruc,mu,infoType)
%         x2 = xTrue(1:12);
%         x2(7:8) = x2(7:8)+uPur_dx;
%         uPurTrue=quadController(xTrue(1:12),zeros(4,1),zeros(3,1),x2,1,zeros(12,1));
%     end
    
    
    
%     %preloading
%     uEvaTempStack=cell(nmod,1);
%     RuStack=cell(nmod,1);
%     uEvaTypeStack=cell(nmod,1);
%     
%     for iT=1:numTargets
%         targetLocation=targetLocationVec{iT};
%         for ij=1:nmod
%             uEvaTemp=[];
%             Ru=[];
%             if strcmp(heurTypeStruc{ij},'vmgt')
%                 vmgtScript;
%                 uEvaTemp=uEvaVMGT;
%                 Ru=0.05*du*eye(4);
%                 uEvaTypeStack{(iT-1)*nmod+ij,1}='nash-vmgt';
%             elseif strcmp(heurTypeStruc{ij},'gt-full')
%                 gtfullScript;
%                 uEvaTemp=uEvaGT;
%                 Ru=0.01*du*eye(4);
%                 uEvaTypeStack{(iT-1)*nmod+ij,1}='nash-full';
%             elseif strcmp(heurTypeStruc{ij},'gt-pm')
%                 loadPointMassControlParams;
%                 Ru=0.01*du*eye(4);
%                 uEvaTypeStack{(iT-1)*nmod+ij,1}='nash-PM';
%             elseif strcmp(heurTypeStruc{ij},'vm')
%                 velmatchScript;
%                 uEvaTemp=uEvaVM;
%                 Ru=0.05*eye(4);
%                 uEvaTypeStack{(iT-1)*nmod+ij,1}='vm';
%             elseif strcmp(heurTypeStruc{ij},'vmgt-heur')
%                 heurtype='both';
%                 vmgt_RA_HeurScript;
%                 uEvaTemp=uEvaVMGTH;
%                 Ru=0.10*eye(4);
%                 uEvaTypeStack{(iT-1)*nmod+ij,1}='nash-vmgt-heur1';
%             elseif strcmp(heurTypeStruc{ij},'vmgt-heur2')
%                 heurtype='heur_only';
%                 vmgt_RA_HeurScript;
%                 uEvaTemp=uEvaVMGTH;
%                 Ru=0.10*eye(4);
%                 uEvaTypeStack{(iT-1)*nmod+ij,1}='nash-vmgt-heur2';
%             elseif strcmp(heurTypeStruc{ij},'other')
%                 uEvaTemp=omega_hover*ones(4,1);
%                 Ru=1*eye(4);
%                 uEvaTypeStack{(iT-1)*nmod+ij,1}='hover';
%             end
%             uEvaTempStack{(iT-1)*nmod+ij,1}=uEvaTemp;
%             RuStack{(iT-1)*nmod+ij,1}=Ru;
%         end
%     end
    
    %    uEvaTrue=uEvaTemp{evaControlType(n,1)};
    
%     if flagUseMeanBestResponse
%         % Generate best responses
%         uPurBestResponseStack=cell(nmod*numTargets,1);
%         for iT=1:numTargets
%             targetLocation=targetLocationVec{iT};
%             for ij=1:nmod
%                 SInput.Spur=Spur; SInput.Seva=Seva;
%                 SInput.Seva.uMat={}; SInput.Seva.uMat{1}=uEvaTempStack{(iT-1)*nmod+ij,1}; SInput.Seva.controlType='gt_overu';
%                 SInput.utemp=utempFine; SInput.uvec=uvecFine; SInput.umax=upmax; %upmax specifically
%                 SInput.gameState=gameState;
%                 gameState.Rtarget.x_target=targetLocation;
%                 SInput.player='pur'; SInput.type='discretize';
%                 u2=optimizeGivenEnemyControl(SInput);
%                 uPurBestResponseStack{(iT-1)*nmod+ij,1}=u2;
%             end
%         end
%     end
    
%     xEndStateMean=zeros(size(xTrue(1:12)));
%     if flagUseMeanBestResponse
%         if strcmp(meanBestResponseType,'mean_rotor')
%             uPurTrue=zeros(4,1);
%             for ij=1:nmod*numTargets
%                 uPurTrue=uPurTrue+mu(ij)*uPurBestResponseStack{ij,1};
%             end
%         elseif strcmp(meanBestResponseType,'mean_output')
%             for ij=1:nmod*numTargets
%                 %NOTE: This adds attitude incorrectly but only position
%                 % states matter to the controller
% %                 muT = mu(ij)
% %                 uT = uPurBestResponseStack{ij,1}
%                 xEndStateMean=xEndStateMean+mu(ij)*f_dynPurQuad(xTrue(1:12),uPurBestResponseStack{ij,1},dt,zeros(2,1));
%             end
%         else
%             error('Unrecognized response type');
%         end
% %     elseif ~flagUseMeanBestResponse
% %         x2 = xTrue(1:12);
% %         uPurTrue=quadController(xTrue(1:12),zeros(4,1),zeros(3,1),x2,1,zeros(12,1));
%     end
%     if flagUseMeanBestResponse && strcmp(meanBestResponseType,'mean_output')
%         uPurTrue=quadController(xTrue(1:12),zeros(4,1),zeros(3,1),xEndStateMean,1,zeros(12,1));
%     end
    
    
    uPhist=[uPhist uPurTrue];
    uEhist=[uEhist uEvaTrue];
    xTrue(1:12)=f_dynPurQuad(xTrue(1:12),uPurTrue(:,1),dt,zeros(2,1));
    xTrue(13:24)=f_dynEvaQuad(xTrue(13:24),uEvaTrue(:,1),dt,zeros(2,1));
    xD=[zeros(24,1) xTrue];
    xPur=xTrue;

    zMeas=xTrue(13:24)+(chol(Rk))'*rand(12,1);
    
    Jpur=feval(Spur.Jname,xD(1:12,:),xD(13:24,:),uPurTrue,uEvaTrue,Spur.Jparams);
    Jeva=feval(Seva.Jname,xD(13:24,:),xD(1:12,:),uEvaTrue,uPurTrue,Seva.Jparams);
    JJp=JJp+Jpur
    JJe=JJe+Jeva
    
%     lambdaTemp=-1*ones(nmod*numTargets,1);
%     for ij=1:nmod*numTargets
%         cc=zeros(numModels,1);
%         for i2=1:numModels
%             ccDum=0;
%             for i3=1:numModels
%                 ccDum=ccDum+Mij(i3,ij)*mu(i3);
%             end
%             cc(i2)=ccDum;
%         end
%         muij=zeros(numModels,1);
%         for j2=1:numModels
%             muij(j2)=Mij(j2,ij)*1/cc(j2)*mu(j2);
%         end
%         uEvaMMF=uEvaTempStack{ij};
%         RuMMF=RuStack{ij};
%         
%         xBarijTemp=zeros(size(xhatE(:,ij)));
%         PijTemp=zeros(size(PhatE(:,:,ij)));
%         for j3=1:numModels
%             xBarijTemp = xBarijTemp+xhatE(:,j3)*muij(j3);
%         end
%         for j4=1:numModels
%             xDeltaTemp = xBarijTemp-xhatE(:,j4);
%             PijTemp = PijTemp+muij(j4)*(PhatE(:,:,j4)+xDeltaTemp*xDeltaTemp');
%         end
%         
%         Paug=PhatE(:,:,ij); Qk=RuMMF;
%         if min(eig(Paug))>0 %check for impossible models
%             % Propagate
%             [xhatEp1,Pp1]=ukfPropagate(xBarijTemp,PijTemp,uEvaMMF,RuMMF,dt,'f_dynEvaQuad');
%             
%             % Measure
%             [xhatE(:,ij),PhatE(:,:,ij),nu,Sk,Wk]=kfMeasure(xhatEp1,Pp1,zMeas,eye(length(ewxvEva)),Rk);
%             Skt=triu(Sk); Skt=Skt'+Skt; Skt(1:13:end)=diag(Sk); %forces symmetry
%             normpEval = mvnpdf(nu,zeros(length(ewxvEva),1),Skt);
%         else
%             normpEval=1e-20;
%         end
%         if normpEval<=1e-8
%             normpdf_eval=1e-8;
%         end
%         lambdaTemp(ij)=normpEval;
%     end
%     muTemp=mu;
%     for ij=1:nmod*numTargets
%         muTemp(ij)=lambdaTemp(ij)*mu(ij)/dot(lambdaTemp,mu);
%         if muTemp(ij)<1e-3
%             muTemp(ij)=1e-3;
%         end
%     end
%     mu=muTemp/sum(muTemp);
%     muHist=[muHist mu];
    
%     xhatE(:,1)-xTrue(13:24)
    
    noise=zeros(24,1);
    xEva=xTrue+noise;
    xPur=xTrue+noise;
    
    xStore=[xStore xTrue];
end

end

