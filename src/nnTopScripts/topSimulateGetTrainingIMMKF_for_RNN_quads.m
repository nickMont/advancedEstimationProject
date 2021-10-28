clear;clc;loadenv;

%
% Game state convention
%        +x
%         |
%         |
%   +y - -
%

%NOTE: CURRENTLY SET FOR U BASED ON VELOCITY MATCHING SCALE


% rngseedno=2;
% rng(rngseedno)
numNNiter=100;


% offset=zeros(8,1);
% offset=[4 4 0 0 4 4 0 0]';

flagRunMMKF=true;

fdyn_trueNameP='f_dynCD2_simple';
fdyn_trueNameE='f_dynCD2_simple';

flagHasTarget=true;
flagPursuerUsesPureVM=true;

qrStore=cell(1,numNNiter);
xStore=cell(1,numNNiter);
measStore=cell(1,numNNiter);
controlTypeStore=cell(1,numNNiter);
uStore=cell(2,numNNiter);
muHistStore=cell(1,numNNiter);
xHatStore=cell(1,numNNiter);
if flagHasTarget==true
    targetStore=cell(1,numNNiter);
    targetTrueIndexStore=cell(1,numNNiter);
    targetPossibleStore=cell(1,numNNiter);
end

numPossibleTargets=4;

for ikN=1:numNNiter

currentStep=ikN/numNNiter*100    

ctrTypeR=floor(rand*3);

% Use best response by taking mean of rotor speeds
flagUseMeanBestResponse=false;
meanBestResponseType='mean_output'; %mean_omega, mean_output
% output: 8.8181e+03
% omega:  8.8181e+03

heurTypeStruc={};
heurTypeStruc{1,1}='gt-full';
heurTypeStruc{2,1}='vmgt';
heurTypeStruc{3,1}='gt-pm';
heurTypeStruc{4,1}='vm';
heurTypeStruc{5,1}='vmgt-heur';
heurTypeStruc{6,1}='other';

randControl=ceil(6*rand);
%map heurTypeStruc{randControl} to something recognized by uEvaType

% General control type flags
uPurType='vm'; %vm OR gt (type of gt specified as controlType)
uEvaType=heurTypeStruc{randControl};
Spur.controlType='gt_overx'; %vmquad, gt_overx
Seva.controlType='gt_overx';
flagRunMMKF=true;
flagRunIMMKF=true; %note: both flagRunMMKF AND flagRunIMMKF must be set true to run IMMKF

scaleVec=0.8; %magnitude of desired uE control relative to uP control
vmtune=0.8; %deceleration parameter for VM

% gameState_p.controlType='gt_overx';
omega_hover=4.95;

nmod=5;

% load nnTrainSets\nnQuadDyn\network.mat
% gameState_p.NN=net;
gameState.tryNN=false;

umax=.5;
uLmax=8; %max low-level control

dt=0.1;
t0=0;
tmax=5;

nt=length(t0:dt:tmax)+1;

% Evader control type (truth parameter)
evaControlType=1*ones(length(0:dt:tmax),1);

%utemp=permn(-2:0.2:2,2)';
du=0.2; %discretization size
uvec=-1:du:1;
utemp=permn(uvec,2)';
upmax=umax;
umax=upmax;
utype.radius=upmax;
utemp=mapUtempToUvec(utemp,"circle",utype);

%Finer version for disc optimizer
uvecFine=-1:du/2:1;
utempFine=permn(uvecFine,2)';
utempFine=mapUtempToUvec(utempFine,"circle",utype);

n=0;
z31=zeros(3,1);
ewxvPur=[z31; z31; rand0(100,2,1);0; rand0(100,2,1);0];
ewxvEva=[z31; z31; rand0(100,2,1);0; rand0(100,2,1);0];
xPur=[ewxvPur;ewxvEva];
xEva=[ewxvPur;ewxvEva];
xTrue=xPur;

targetLocation=[rand(10,2,1);0];
QtargetP=diag(rand0(100,3,1));
QtargetE=diag(rand0(100,3,1));
numTargets=1;

Qpur=zeros(12,12);
Qpur(7:9,7:9)=15*diag([5,5,0]);
Qeva=zeros(12,12);
Qeva(7:8,7:8)=15*eye(2);
Rpur=eye(4);
Reva=eye(4);
xStore=xPur;
Spur.Jparams.Q_target=QtargetP;
Spur.Jparams.x_target=targetLocation;
Seva.Jname='J_evaQuad';
Seva.fname='f_dynEvaQuadTarget';
Seva.Jparams.Q_target=QtargetE;
Seva.Jparams.x_target=targetLocation;
Seva.UseVelMatch=true;

gameState.Rtarget.useNoHeuristics=false;
gameState.Rtarget.useMotionPrediction=true;

Spur.Jname='J_purQuadTarget';
Spur.fname='f_dynPurQuad';
Seva.Jname='J_evaQuadTarget';
Seva.fname='f_dynEvaQuad';


uPhist=[];
uEhist=[];

JJp=0;
JJe=0;

xhatE=repmat(ewxvEva,[1 nmod]);
PhatE=repmat(0.001*eye(length(ewxvEva)),[1 1 nmod]);
mu=1/nmod*ones(nmod,1);
muHist=mu;
Rk=0.1*eye(length(ewxvEva));
if flagRunIMMKF
    Mij=[0.96 0.01 0.01 0.01 0.01
        0.01 0.96 0.01 0.01 0.01
        0.01 0.01 0.96 0.01 0.01
        0.01 0.01 0.01 0.96 0.01
        0.01 0.01 0.01 0.01 0.96];
    [aMij,bMij]=size(Mij);
    if aMij~=nmod || bMij~=nmod
        error('Mij size does not match number of models')
    end
end

xhatHist=[];

tic
for t=t0:dt:tmax
    n=n+1;
    
    Spur.Jname='J_purQuad';
    Seva.Jname='J_evaQuad';
    Spur.Jparams.Q=Qpur;
    Spur.Jparams.Rself=Rpur;
    Spur.Jparams.Ropp=zeros(4,4);
    Spur.uLmax=uLmax;
    Spur.utemp=utemp;
    Spur.du=du;
    Spur.uvec=uvec;
    Spur.uEvaEstForVM=[0;0];
    Spur.VMparams.vmtune=vmtune;
    Spur.VMparams.scalevec=scaleVec;
    Seva.Jparams.Q=Qeva;
    Seva.Jparams.Rself=Reva;
    Seva.Jparams.Ropp=zeros(4,4);
    Seva.uLmax=uLmax;
    Seva.utemp=utemp;
    Seva.du=du;
    Seva.uvec=uvec;
    % Robust estimation params
    miscParams.Qk=0.001*eye(24);
    miscParams.useUT=false;
    
    % Guessing pursuer
    gameState.xPur=xPur(1:12);
    gameState.xEva=xPur(13:24);
    gameState.dt=dt;
    gameState.kMax=1;
    gameState.nu=2;
    gameState.discType='overX';
    gameState.uMaxP=umax;
    gameState.Rtarget.x_target=targetLocation;
    Spur.uMat={0}; Seva.uMat={0};
    if strcmp(Spur.controlType,'vmquad')
        for ik=1:length(uvec)
            Spur.uMat{ik}=upmax*uvec(ik);
        end
    end
    if strcmp(Spur.controlType,'gt_overx')
        for ik=1:length(utemp)
            Spur.uMat{ik}=utemp(:,ik);
        end
    end
    Spur.Jname='J_purQuadTarget';
    Spur.fname='f_dynPurQuad';
    Spur.UseVelMatch=true;
    if strcmp(Seva.controlType,'vmquad')
        for ik=1:length(uvec)
            Seva.uMat{ik}=upmax*uvec(ik);
        end
    end
    if strcmp(Seva.controlType,'gt_overx')
        for ik=1:length(utemp)
            Seva.uMat{ik}=utemp(:,ik);
        end
    end
    Spur.Jparams.Q_target=QtargetP;
    Spur.Jparams.x_target=targetLocation;
    Seva.Jname='J_evaQuadTarget';
    Seva.fname='f_dynEvaQuad';
    Seva.Jparams.Q_target=QtargetE;
    Seva.Jparams.x_target=targetLocation;
    Seva.UseVelMatch=true;
    
    if strcmp(uEvaType,'gt')||strcmp(uPurType,'gt')
        % Propagate to next time step
        [up,ue,flag,uPSampled,uESampled,Sminimax,Smisc]=f_dyn2(Spur,Seva,gameState,zeros(4,1),miscParams);
        if strcmp(uPurType,'gt')
            uPurTrue=uPSampled;
        end
        if strcmp(uEvaType,'gt')
            uEvaTrue=uESampled;
            uEvaNash=uESampled;
        end
    elseif strcmp(uEvaType,'vm')||strcmp(uPurType,'vm')
        %velmatchScript
        xp=[xTrue(7:8);xTrue(10:11)];xe=[xTrue(19:20);xTrue(22:23)];
        uP=vmRGVO_tune(xp,xe,umax,2,dt,zeros(2,1),vmtune);
        uE=-scaleVec*uP;
        xd = xPur(13:24); xd(9)=0;
        if strcmp(uEvaType,'vm')
            uEvaTrue = quadControllerACCONLY(xd, zeros(4,1), 3, [uE;0],0);
        end
        xd = xPur(1:12); xd(9)=0;
        if strcmp(uPurType,'vm')
            uPurTrue = quadControllerACCONLY(xd, zeros(4,1), 3, [uP;0],0);
        end
    else
        error('No controller specified')
    end
    
    
    gameStateTemp=gameState;
    gameStateTemp.miscParams=miscParams;
    gameStateTemp.Rtarget.x_target = targetLocation;
    % overwrite true control
    uEvaTrue=getUForMMKF(Spur,Seva,gameStateTemp,heurTypeStruc{randControl,1},true);
    if strcmp('other',heurTypeStruc{randControl})
        uEvaTrue=uEvaTrue+0.5*randn(4,1);
    end
    
    
    
    uEvaTempStack=cell(nmod,1);
    RuStack=cell(nmod,1);
    uEvaTypeStack=cell(nmod,1);
    
    if flagRunMMKF
    for ij=1:nmod
        gameStateTemp=gameState;
        gameStateTemp.miscParams=miscParams;
        gameStateTemp.Rtarget.x_target = targetLocation;
        [uEvaTemp,Ru,uEvaTypeTemp] = getUForMMKF(Spur,Seva,gameStateTemp,heurTypeStruc{ij,1},true);
%         uEvaTemp=[];
%         Ru=[];
%         if ij==1
%             vmgtScript;
%             uEvaTemp=uEvaVMGT;
%             Ru=0.05*du*eye(4);
%             uEvaTypeStack{ij,1}='nash-vmgt';
%         elseif ij==2
%             gtfullScript;
%             uEvaTemp=uEvaGT;
%             Ru=0.01*du*eye(4);
%             uEvaTypeStack{ij,1}='nash-full';
%         elseif ij==3
%             loadPointMassControlParams;
%             Ru=0.01*du*eye(4);
%             uEvaTypeStack{ij,1}='nash-PM';
%         elseif ij==4
%             velmatchScript;
%             uEvaTemp=uEvaVM;
%             Ru=0.05*eye(4);
%             uEvaTypeStack{ij,1}='vm';
%         elseif ij==5
%             uEvaTemp=omega_hover*ones(4,1);
%             Ru=1*eye(4);
%             uEvaTypeStack{ij,1}='hover';
%         end
        uEvaTypeStack{ij,1}=uEvaTypeTemp;
        uEvaTempStack{ij,1}=uEvaTemp;
        RuStack{ij,1}=Ru;
    end
    end
    
%    uEvaTrue=uEvaTemp{evaControlType(n,1)};
    
    if flagRunMMKF
    % Generate best responses
    uPurBestResponseStack=cell(nmod,1);
    for ij=1:nmod
        if strcmp(uEvaTypeStack{ij,1},'nash')
            u2=uPurTrue;
        elseif strcmp(uEvaTypeStack{ij,1},'minimax')
            u2=Sminimax.uP;
        else
            SInput.Spur=Spur; SInput.Seva=Seva;
            SInput.Seva.uMat={}; SInput.Seva.uMat{1}=uEvaTempStack{ij,1}; SInput.Seva.controlType='gt_overu';
            SInput.utemp=utempFine; SInput.uvec=uvecFine; SInput.umax=umax;
            SInput.gameState=gameState;
            SInput.player='pur'; SInput.type='discretize';
            u2=optimizeGivenEnemyControl(SInput);
        end
        uPurBestResponseStack{ij,1}=u2;
    end
    end
    
    if flagRunMMKF
    xEndStateMean=zeros(size(xTrue(1:12)));
    if flagUseMeanBestResponse
        if strcmp(meanBestResponseType,'mean_rotor')
            uPurTrue=zeros(4,1);
            for ij=1:nmod
                uPurTrue=uPurTrue+mu(ij)*uPurBestResponseStack{ij,1};
            end
        elseif strcmp(meanBestResponseType,'mean_output')
            for ij=1:nmod
                %NOTE: This adds attitude incorrectly but only position
                % states matter to the controller
                xEndStateMean=xEndStateMean+mu(ij)*f_dynPurQuad(xTrue(1:12),uPurBestResponseStack{ij,1},dt,zeros(2,1));
            end
        end
    end
    if flagUseMeanBestResponse && strcmp(meanBestResponseType,'mean_output')
        uPurTrue=quadController(xTrue(1:12),zeros(4,1),zeros(3,1),xEndStateMean,1,zeros(12,1));
    end
    end
    
    
    uPhist=[uPhist uPurTrue];
    uEhist=[uEhist uEvaTrue];
    xTrue(1:12)=f_dynPurQuad(xTrue(1:12),uPurTrue(:,1),dt,zeros(2,1));
    xTrue(13:24)=f_dynEvaQuad(xTrue(13:24),uEvaTrue(:,1),dt,zeros(2,1));
    xD=[zeros(24,1) xTrue];

    zMeas=xTrue(13:24)+chol(Rk)*rand(12,1);
    
    Jpur=feval(Spur.Jname,xD(1:12,:),xD(13:24,:),uPurTrue,uEvaTrue,Spur.Jparams);
    Jeva=feval(Seva.Jname,xD(13:24,:),xD(1:12,:),uEvaTrue,uPurTrue,Seva.Jparams);
    JJp=JJp+Jpur
    JJe=JJe+Jeva
    
    if flagRunMMKF
    lambdaTemp=-1*ones(nmod,1);
    for ij=1:nmod
        uEvaMMF=uEvaTempStack{ij};
        RuMMF=RuStack{ij};
        
        Paug=PhatE(:,:,ij); Qk=RuMMF;
        if min(eig(Paug))>0 %check for impossible models
            % Propagate
            [xhatEp1,Pp1]=ukfPropagate(xhatE(:,ij),PhatE(:,:,ij),uEvaMMF,RuMMF,dt,'f_dynEvaQuad');
            
            % Measure
            [xhatE(:,ij),PhatE(:,:,ij),nu,Sk,Wk]=kfMeasure(xhatEp1,Pp1,zMeas,eye(length(ewxvEva)),Rk);
            Skt=triu(Sk); Skt=Skt'+Skt; Skt(1:13:end)=diag(Sk); %forces symmetry
            normpEval= mvnpdf(nu,zeros(length(ewxvEva),1),Skt);
        else
            normpEval=1e-20;
        end
        if normpEval<=1e-8
            normpdf_eval=1e-8;
        end
        lambdaTemp(ij)=normpEval;
    end
    muTemp=mu;
    for ij=1:nmod
        muTemp(ij)=lambdaTemp(ij)*mu(ij)/dot(lambdaTemp,mu);
        if muTemp(ij)<1e-8
            muTemp(ij)=1e-8;
        end
    end
    mu=muTemp/sum(muTemp);
    muHist=[muHist mu];
    end
    
    xhatHist=[xhatHist sum(mu'.*xhatE)];
    
%     xhatE(:,1)-xTrue(13:24)
    
    noise=zeros(24,1);
    xEva=xTrue+noise;
    xPur=xTrue+noise;
    
    xStore=[xStore xTrue];
end
tTotal=toc

[maxv,maxdex]=max(mu);
ntar=mod(maxdex,numTargets)+1;
nctr=floor(maxdex/nmod);

if nctr==nctrTypeR && xTargetIndexTrue==ntar
    xStore{ikN}=xBt;
    uStore{1,ikN}=uBp;
    uStore{2,ikN}=uBe;
    measStore{ikN}=zB;
    muHistStore{ikN}=muHist;
    xHatStore{ikN}=xhatHist;
else
    ikN=ikN-1;
end




if plotEndFlag==1
    xP=zeros(2,n+1);xE=zeros(2,n+1);
    for ikk=1:n+1
        xP(:,ikk)=xTrueS{ikk}(1:2); xE(:,ikk)=xTrueS{ikk}(5:6);
    end
    figure(3);clf;
    figset
    plot(xP(1,:),xP(2,:),'-xr');
    hold on
    plot(xE(1,:),xE(2,:),'-ob');
    title('Interceptor using velocity matching vs unaware evader');
    xlabel('East displacement (m)');
    ylabel('North displacement (m)');
    legend('Pursuer','Evader','Location','Southeast');
%     axis(axisveck)
    figset
    
    figure(4); clf;
    subplot(2,1,1)
    figset
    plot(0:tstep:tmax+tstep,xP(1,:),'-xr')
    hold on
    plot(0:tstep:tmax+tstep,xE(1,:),'-ob')
    xlabel('Time (s)')
    ylabel('East displacement (m)')
    legend('Pursuer','Evader','Location','Southeast');
    figset
    subplot(2,1,2)
    figset
    plot(0:tstep:tmax+tstep,xP(2,:),'-xr')
    hold on
    plot(0:tstep:tmax+tstep,xE(2,:),'-ob')
    xlabel('Time (s)')
    ylabel('East displacement (m)')
    legend('Pursuer','Evader','Location','Southeast');
    figset
    
end


end

flagUsedVMforPur=flagPursuerUsesPureVM