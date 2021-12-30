clear;clc;loadenv;
beep off;

%note use addpath('name') to add folders to path

rngseedno=460;
rng(rngseedno);

%NOTE: TUNING IN VMRGO REQUIRES USING fdynEva AND fdynPur FOR PM SIM
%  SET DRAG PARAMS IN BOTH TO 0.
%NOTE: set the target function to generateCostMatricesVMquad in f_dyn2
%  BEFORE running this; hardcoded the other way to remain backwards
%  compatible

% mean_output
% JJp =
%    6.4783e+03
% JJe =
%    4.7913e+05
% tTotal =
%   279.8138
% mean_rotor
% JJp =
%    5.3687e+03
% JJe =
%    4.7941e+05

% mean_output
% JJp =
%    1.6489e+04
% JJe =
%    1.5958e+06
%    
% mean_rotor
% JJp =
%    1.7537e+04
% JJe =
%    1.5977e+06

%high control cost
% mean_output
% JJp =
%    4.8287e+04
% JJe =
%    1.5958e+06
% mean_rotor
% JJp =
%    4.9275e+04
% JJe =
%    1.5977e+06

% Use best response by taking mean of rotor speeds
flagUseMeanBestResponse=true;
meanBestResponseType='mean_output'; %mean_omega, mean_output
% General control type flags
FLAG_useGameTheoreticController=true;
FLAG_usePureVelMatchController=false;
FLAG_tryMotionPredictionInVM2=false; %try heuristic for modifying motion direction prediction
FLAG_tryVMGTbutBypassHeuristics=false;

numRefinements=0; %number of refinements for VM heading
% output: 8.8181e+03
% omega:  8.8181e+03

scaleVec=0.8; %magnitude of desired uE control relative to uP control
vmtune=0.8; %deceleration parameter for VM

% Control type flags, if GT specified
% select from: vmquad, gt_overx
Spur.controlType='vmquad';
Seva.controlType='vmquad';
% gameState_p.controlType='gt_overx';
omega_hover=4.95;

heurTypeStruc{1}='vmgt';
heurTypeStruc{2}='gt-full';
heurTypeStruc{3}='gt-pm';
heurTypeStruc{4}='vm';
heurTypeStruc{5}='vmgt-heur';
% heurTypeStruc{6}='other';
[~,nmod]=size(heurTypeStruc);

% load nnTrainSets\nnQuadDyn\network.mat
% gameState_p.NN=net;
gameState.tryNN=false;

umax=.5;
uLmax=8; %max low-level control

dt=0.1;
t0=0;
tmax=5;

% Evader control type (truth parameter)
evaControlType=1*ones(length(0:dt:tmax),1);

%utemp=permn(-2:0.2:2,2)';
du=0.5; %discretization size
uvec=-1:du:1;
utemp=permn(uvec,2)';
upmax = 2*umax;
uemax = umax;
utype.radius=upmax;
utemp=mapUtempToUvec(utemp,"circle",utype);

%Finer version for disc optimizer
uvecFine=-1:du/2:1;
utempFine=permn(uvecFine,2)';
utempFine=mapUtempToUvec(utempFine,"circle",utype);

n=0;
z31=zeros(3,1);
ewxvPur=[z31; z31; 1;0.1;0; z31];
ewxvEva=[zeros(12,1)];
xPur=[ewxvPur;ewxvEva];
xEva=[ewxvPur;ewxvEva];
xTrue=xPur;

numTargets=2;
purTrueTargetIndex=1;
targetLocationVec{1}=[-5;-5;0];
targetLocationVec{2}=[5;0;0];
QtargetP = 0*diag([1;1;0]);
QtargetE = 10*diag([1;1;0]);
Qpur=zeros(12,12); Qeva=zeros(12,12);
Qpur(7:8,7:8)=50*eye(2);
Qeva(7:8,7:8)=1*eye(2);
Rpur=eye(4);
Reva=eye(4);
xStore=xPur;

uPhist=[];
uEhist=[];

JJp=0;
JJe=0;

xhatP=repmat(ewxvPur,[1 nmod*numTargets]);
PhatP=repmat(0.001*eye(length(ewxvEva)),[1 1 nmod*numTargets]);
mu=1/(nmod*numTargets)*ones(nmod*numTargets,1);
muHist=mu;
Rk=0.1*eye(length(ewxvEva));

numModels=nmod*numTargets;
Mij=eye(numModels)+0.01*ones(numModels);
[aMij,bMij]=size(Mij);
for ij=1:aMij
    Mij(ij,:) = Mij(ij,:)/sum(Mij(ij,:));
end
if aMij~=nmod*numTargets || bMij~=nmod*numTargets
    error('Mij size does not match number of models')
end

tic
for t=t0:dt:tmax
    tLoop = t
    n=n+1;
    
    Spur.Jname='J_purQuadTarget';
    Seva.Jname='J_evaQuadTarget';
    Spur.Jparams.Q=Qpur;
    Spur.Jparams.Rself=Rpur;
    Spur.Jparams.Ropp=zeros(4,4);
    Spur.uLmax=uLmax;
    Seva.Jparams.Q=Qeva;
    Seva.Jparams.Rself=Reva;
    Seva.Jparams.Ropp=zeros(4,4);
    Seva.uLmax=uLmax;
    
    % Robust estimation params
    miscParams.Qk=0.001*eye(24);
    miscParams.useUT=false;
    
    
    uEvaEst=zeros(2,1);
    
    % Guessing pursuer
    gameState.xPur=xPur(1:12);
    gameState.xEva=xPur(13:24);
    gameState.dt=dt;
    gameState.kMax=1;
    gameState.nu=2;
    gameState.discType='overX';
    gameState.uMaxP=upmax;
    gameState.uEvaEstForVM=uEvaEst;
    Spur.uMat={}; Seva.uMat={};
    if strcmp(Spur.controlType,'vmquad')
        for ik=1:length(uvec)
            Spur.uMat{ik}=upmax*uvec(ik);
        end
    end
    if strcmp(Spur.controlType,'gt_overx')
        for ik=1:length(utemp)
            Spur.uMat{ik}=upmax*utemp(:,ik);
        end
    end
    Spur.Jname='J_purQuadTarget';
    Spur.fname='f_dynPurQuad';
    Spur.UseVelMatch=true;
    if strcmp(Seva.controlType,'vmquad')
        for ik=1:length(uvec)
            Seva.uMat{ik}=uemax*uvec(ik);
        end
    end
    if strcmp(Seva.controlType,'gt_overx')
        for ik=1:length(utemp)
            Seva.uMat{ik}=uemax*utemp(:,ik);
        end
    end
    Spur.Jparams.Q_target=QtargetP;
    Seva.Jname='J_evaQuadTarget';
    Seva.fname='f_dynEvaQuad';
    Seva.Jparams.Q_target=QtargetE;
    
    gameState.uEvaEstForVM=uEvaEst;
    gameState.Rtarget.Q_target=QtargetP;
    gameState.Rtarget.useMotionPrediction=FLAG_tryMotionPredictionInVM2;
    gameState.Rtarget.useNoHeuristics=FLAG_tryVMGTbutBypassHeuristics;
    Seva.UseVelMatch=true;
    
    %     if t>3
    %         fprintf('manual swap at line 150')
    %         evaTrueTargetIndex=2;
    %     end
    
    for iT=1:numTargets
        targetLocation=targetLocationVec{iT};
        if FLAG_useGameTheoreticController
            for iR=1:numRefinements+1
                gameState.Rtarget.x_target=targetLocation;
                Seva.Jparams.x_target=targetLocation;
                Spur.Jparams.x_target=targetLocation;
                % Propagate to next time step
                [up,ue,flag,uPSampled,uESampled,Sminimax,Smisc]=f_dyn2(Spur,Seva,gameState,zeros(4,1),miscParams);
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
    uPurTrue=uPurTrueStr{purTrueTargetIndex};
    uEvaTrue=uEvaTrueStr{1};
    
    %preloading
    uPurTempStack=cell(nmod,1);
    RuStack=cell(nmod,1);
    uPurTypeStack=cell(nmod,1);
    
    for iT=1:numTargets
        targetLocation=targetLocationVec{iT};
        for ij=1:nmod
            uPurTemp=[];
            Ru=[];
            if strcmp(heurTypeStruc{ij},'vmgt')
                vmgtScript;
                uPurTemp=uPurVMGT;
                Ru=0.05*du*eye(4);
                uPurTypeStack{(iT-1)*nmod+ij,1}='nash-vmgt';
            elseif strcmp(heurTypeStruc{ij},'gt-full')
                gtfullScript;
                uPurTemp=uPurGT;
                Ru=0.01*du*eye(4);
                uPurTypeStack{(iT-1)*nmod+ij,1}='nash-full';
            elseif strcmp(heurTypeStruc{ij},'gt-pm')
                loadPointMassControlParams;
                Ru=0.01*du*eye(4);
                uPurTypeStack{(iT-1)*nmod+ij,1}='nash-PM';
            elseif strcmp(heurTypeStruc{ij},'vm')
                velmatchScript;
                uPurTemp=uPurVM;
                Ru=0.05*eye(4);
                uPurTypeStack{(iT-1)*nmod+ij,1}='vm';
            elseif strcmp(heurTypeStruc{ij},'vmgt-heur')
                heurtype='both';
                vmgt_RA_HeurScript;
                uPurTemp=uPurVMGTH;
                Ru=0.10*eye(4);
                uPurTypeStack{(iT-1)*nmod+ij,1}='nash-vmgt-heur1';
            elseif strcmp(heurTypeStruc{ij},'vmgt-heur2')
                heurtype='heur_only';
                vmgt_RA_HeurScript;
                uPurTemp=uPurVMGTH;
                Ru=0.10*eye(4);
                uPurTypeStack{(iT-1)*nmod+ij,1}='nash-vmgt-heur2';
            elseif strcmp(heurTypeStruc{ij},'other')
                uPurTemp=omega_hover*ones(4,1);
                Ru=1*eye(4);
                uPurTypeStack{(iT-1)*nmod+ij,1}='hover';
            end
            uPurTempStack{(iT-1)*nmod+ij,1}=uPurTemp;
            RuStack{(iT-1)*nmod+ij,1}=Ru;
        end
    end
    
%     %    uEvaTrue=uEvaTemp{evaControlType(n,1)};
%     
%     % Generate best responses
%     uPurBestResponseStack=cell(nmod*numTargets,1);
%     for iT=1:numTargets
%         targetLocation=targetLocationVec{iT};
%         for ij=1:nmod
%             SInput.Spur=Spur; SInput.Seva=Seva;
%             SInput.Seva.uMat={}; SInput.Seva.uMat{1}=uPurTempStack{(iT-1)*nmod+ij,1}; SInput.Seva.controlType='gt_overu';
%             SInput.utemp=utempFine; SInput.uvec=uvecFine; SInput.umax=upmax; %upmax specifically
%             SInput.gameState=gameState;
%             gameState.Rtarget.x_target=targetLocation;
%             SInput.player='pur'; SInput.type='discretize';
%             u2=optimizeGivenEnemyControl(SInput);
%             uPurBestResponseStack{(iT-1)*nmod+ij,1}=u2;
%         end
%     end
%     
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
%     end
%     if flagUseMeanBestResponse && strcmp(meanBestResponseType,'mean_output')
%         uPurTrue=quadController(xTrue(1:12),zeros(4,1),zeros(3,1),xEndStateMean,1,zeros(12,1));
%     end
    
    
    uPhist=[uPhist uPurTrue];
    uEhist=[uEhist uEvaTrue];
    xTrue(1:12)=f_dynPurQuad(xTrue(1:12),uPurTrue(:,1),dt,zeros(2,1));
    xTrue(13:24)=f_dynEvaQuad(xTrue(13:24),uEvaTrue(:,1),dt,zeros(2,1));
    xD=[zeros(24,1) xTrue];

    zMeas=xTrue(1:12)+chol(Rk)*rand(12,1);
    
    Jpur=feval(Spur.Jname,xD(1:12,:),xD(13:24,:),uPurTrue,uEvaTrue,Spur.Jparams);
    Jeva=feval(Seva.Jname,xD(13:24,:),xD(1:12,:),uEvaTrue,uPurTrue,Seva.Jparams);
    JJp=JJp+Jpur
    JJe=JJe+Jeva
    
    lambdaTemp=-1*ones(nmod*numTargets,1);
    for ij=1:nmod*numTargets
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
        uEvaMMF=uPurTempStack{ij};
        RuMMF=RuStack{ij};
        
        xBarijTemp=zeros(size(xhatP(:,ij)));
        PijTemp=zeros(size(PhatP(:,:,ij)));
        for j3=1:numModels
            xBarijTemp = xBarijTemp+xhatP(:,j3)*muij(j3);
        end
        for j4=1:numModels
            xDeltaTemp = xBarijTemp-xhatP(:,j4);
            PijTemp = PijTemp+muij(j4)*(PhatP(:,:,j4)+xDeltaTemp*xDeltaTemp');
        end
        
        Paug=PhatP(:,:,ij); Qk=RuMMF;
        if min(eig(Paug))>0 %check for impossible models
            % Propagate
            [xhatEp1,Pp1]=ukfPropagate(xBarijTemp,PijTemp,uEvaMMF,RuMMF,dt,'f_dynEvaQuad');
            
            % Measure
            [xhatP(:,ij),PhatP(:,:,ij),nu,Sk,Wk]=kfMeasure(xhatEp1,Pp1,zMeas,eye(length(ewxvEva)),Rk);
            Skt=triu(Sk); Skt=Skt'+Skt; Skt(1:13:end)=diag(Sk); %forces symmetry
            normpEval = mvnpdf(nu,zeros(length(ewxvEva),1),Skt);
        else
            normpEval=1e-20;
        end
        if normpEval<=1e-8
            normpdf_eval=1e-8;
        end
        lambdaTemp(ij)=normpEval;
    end
    muTemp=mu;
    for ij=1:nmod*numTargets
        muTemp(ij)=lambdaTemp(ij)*mu(ij)/dot(lambdaTemp,mu);
        if muTemp(ij)<1e-3
            muTemp(ij)=1e-3;
        end
    end
    mu=muTemp/sum(muTemp);
    muHist=[muHist mu];
    
%     xhatE(:,1)-xTrue(13:24)
    
    noise=zeros(24,1);
    xEva=xTrue+noise;
    xPur=xTrue+noise;
    
    xStore=[xStore xTrue];
end
tTotal=toc

% %     load quaddat.mat;
% indsamp=1:5:50;
% xP2d=xStore(7:8,indsamp);
% xE2d=xStore(19:20,indsamp);
% % mx=(xP2d(2,1)-xE2d(2,1))/(xP2d(1,1)-xE2d(1,1));
% % xPlin_x=-5:0.3:1;
% % xPlin_y=xPlin_x*mx;
% % xElin_x=-6:0.3:0;
% % xElin_y=xElin_x*mx;
% 
% figure(1);clf;
% figset
% plot(xP2d(1,:),xP2d(2,:),'-*k');
% hold on
% plot(xE2d(1,:),xE2d(2,:),'-Ok');
% % hold on
% % plot(xPlinCompare(1,:),xPlinCompare(2,:),'-.*k');
% % hold on
% % plot(xElinCompare(1,:),xElinCompare(2,:),'-.og');
% axis([-6 2 -6 2])
% xlabel('x-position (m)')
% ylabel('y-position (m)')
% legend('Pursuer trajectory','Evader trajectory');
% figset


figure(3);clf;
figset
plot(xStore(7,:),xStore(8,:),'-.k')
hold on
plot(xStore(19,:),xStore(20,:),'-*k')
figset
xlabel('East displacement (m)')
ylabel('North displacement (m)')
legend('Pursuer trajectory','Evader trajectory')
figset

%<<>> clean this up
figure(2);clf;
figset
plottype=['k-x','k-*','k-0'];
plot(dt*(0:n),muHist(1,:),'k-x')
hold on
plot(dt*(0:n),muHist(2,:),'k-*')
hold on
if nmod>=3
plot(dt*(0:n),muHist(3,:),'k-o')
end
% if nmod>=4
% plot(dt*(0:n),muHist(4,:),'k-+')
% end
% if nmod>=5
% plot(dt*(0:n),muHist(5,:),'k-s')
% end
% if nmod>=6
% plot(dt*(0:n),muHist(6,:),'k-^')
% end
% if numModels>=7
% plot(dt*(0:n),muHist(7,:),'k-v')
% end
hold on
plot(dt*(0:n),muHist(6,:),'k-v')
hold on
plot(dt*(0:n),muHist(7,:),'k-^')
hold on
plot(dt*(0:n),muHist(8,:),'k-+')
figset
xlabel('Time Elapsed (s)')
ylabel('Model Probability')
plot(dt*(0:n),sum(muHist([4 5 9 10],:)),'k-s')
% legend('VM/GT','GT','GT/PM','VM','VMGTHeur','other','VM/GT 2')
legend('VM/GT 1','GT 1','GT/PM 1','VM/GT 2','GT 2','GT/PM 2','others')
figset
%legend('Nash strategy','Non-Nash strategy') %update per side


