clear;clc;loadenv;
beep off;

%note use addpath('name') to add folders to path

rngseedno=457;
rng(rngseedno);

%NOTE: TUNING IN VMRGO REQUIRES USING fdynEva AND fdynPur FOR PM SIM
%  SET DRAG PARAMS IN BOTH TO 0.
%NOTE: set the target function to generateCostMatricesVMquad in f_dyn2
%  BEFORE running this; hardcoded the other way to remain backwards
%  compatible

% Use best response by taking mean of rotor speeds
flagUseMeanBestResponse=false;
meanBestResponseType='mean_output'; %mean_omega, mean_output
% General control type flags
FLAG_useGameTheoreticController=true;
FLAG_usePureVelMatchController=false;
FLAG_tryMotionPredictionInVM2=false; %try heuristic for modifying motion direction prediction
numRefinements=0; %number of refinements for VM heading
% output: 8.8181e+03
% omega:  8.8181e+03

% TEST: no heuristic, Q=diag(5)
% JJp =
%    8.8522e+03
% JJe =
%    4.6839e+04
% tTotal =
%   119.7137
% TEST: with heuristic, Q=diag(5)
% JJp =
%    8.9029e+03
% JJe =
%    4.5657e+04
% tTotal =
%   121.0195

% TEST: no heuristic, Q=diag(10)
% JJp =
%    8.8522e+03
% JJe =
%    8.9503e+04
% tTotal =
%   122.6334
% TEST: with heuristic, Q=diag(10)
% JJp =
%    8.8682e+03
% JJe =
%    8.7177e+04
% tTotal =
%   120.3313
  
  
scaleVec=0.8; %magnitude of desired uE control relative to uP control
vmtune=0.8; %deceleration parameter for VM

% Control type flags, if GT specified
% select from: vmquad, gt_overx
Spur.controlType='vmquad';
Seva.controlType='vmquad';
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
ewxvPur=[z31; z31; 1;0.1;0; z31];
ewxvEva=[zeros(12,1)];
xPur=[ewxvPur;ewxvEva];
xEva=[ewxvPur;ewxvEva];
xTrue=xPur;

targetLocation=[-10;-10;0];
QtargetP=diag([0;0;0]);
QtargetE=diag([10;10;0]);
Qpur=zeros(12,12);
Qpur(7:9,7:9)=15*diag([5,5,0]);
Qeva=zeros(12,12);
Qeva(7:8,7:8)=15*eye(2);
Rpur=eye(4);
Reva=eye(4);
xStore=xPur;

uPhist=[];
uEhist=[];

JJp=0;
JJe=0;

xhatE=repmat(ewxvEva,[1 nmod]);
PhatE=repmat(0.001*eye(length(ewxvEva)),[1 1 nmod]);
mu=1/nmod*ones(nmod,1);
muHist=mu;
Rk=0.1*eye(length(ewxvEva));

tic
for t=t0:dt:tmax
    n=n+1;
    
    Spur.Jname='J_purQuad';
    Seva.Jname='J_evaQuad';
    Spur.Jparams.Q=Qpur;
    Spur.Jparams.Rself=Rpur;
    Spur.Jparams.Ropp=zeros(4,4);
    Spur.uLmax=uLmax;
    Seva.Jparams.Q=Qeva;
    Seva.Jparams.Rself=Reva;
    Seva.Jparams.Ropp=zeros(4,4);
    Seva.uLmax=uLmax;
    
    uEvaEst=zeros(2,1);
    
    if FLAG_useGameTheoreticController
        for iR=1:numRefinements+1
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
            gameState.uEvaEstForVM=uEvaEst;
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
            gameState.uEvaEstForVM=uEvaEst;
            gameState.Rtarget.Q_target=QtargetP;
            gameState.Rtarget.x_target=targetLocation;
            gameState.Rtarget.useMotionPrediction=FLAG_tryMotionPredictionInVM2;
            Seva.UseVelMatch=true;
            % Propagate to next time step
            [up,ue,flag,uPSampled,uESampled,Sminimax,Smisc]=f_dyn2(Spur,Seva,gameState,zeros(4,1),miscParams);
            uPurTrue=uPSampled;
            uEvaTrue=uESampled;
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
        uEvaTrue = quadControllerACCONLY(xd, zeros(4,1), 3, [uE;0],0);
        xd = xPur(1:12); xd(9)=0;
        uPurTrue = quadControllerACCONLY(xd, zeros(4,1), 3, [uP;0],0);
    else
        error('No controller specified')
    end
    
    uEvaTempStack=cell(nmod,1);
    RuStack=cell(nmod,1);
    uEvaTypeStack=cell(nmod,1);
    
    for ij=1:nmod
        uEvaTemp=[];
        Ru=[];
        if ij==1
            vmgtScript;
            uEvaTemp=uEvaVMGT;
            Ru=0.05*du*eye(4);
            uEvaTypeStack{ij,1}='nash-vmgt';
        elseif ij==2
            gtfullScript;
            uEvaTemp=uEvaGT;
            Ru=0.01*du*eye(4);
            uEvaTypeStack{ij,1}='nash-full';
        elseif ij==3
            loadPointMassControlParams;
            Ru=0.01*du*eye(4);
            uEvaTypeStack{ij,1}='nash-PM';
        elseif ij==4
            velmatchScript;
            uEvaTemp=uEvaVM;
            Ru=0.05*eye(4);
            uEvaTypeStack{ij,1}='vm';
        elseif ij==5
            uEvaTemp=omega_hover*ones(4,1);
            Ru=1*eye(4);
            uEvaTypeStack{ij,1}='hover';
        end
        uEvaTempStack{ij,1}=uEvaTemp;
        RuStack{ij,1}=Ru;
    end
    
%    uEvaTrue=uEvaTemp{evaControlType(n,1)};
    
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
                xEndStateMean=1/mu(ij)*f_dynPurQuad(xTrue(1:12),uPurBestResponseStack{ij,1},dt,zeros(2,1));
            end
        end
    end
    if flagUseMeanBestResponse && strcmp(meanBestResponseType,'mean_output')
        uPurTrue=quadController(xTrue(1:12),zeros(4,1),zeros(3,1),xEndStateMean,1,zeros(12,1));
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
    
%     xhatE(:,1)-xTrue(13:24)
    
    noise=zeros(24,1);
    xEva=xTrue+noise;
    xPur=xTrue+noise;
    
    xStore=[xStore xTrue];
end
tTotal=toc

%     load quaddat.mat;
indsamp=1:5:50;
xP2d=xStore(7:8,indsamp);
xE2d=xStore(19:20,indsamp);
% mx=(xP2d(2,1)-xE2d(2,1))/(xP2d(1,1)-xE2d(1,1));
% xPlin_x=-5:0.3:1;
% xPlin_y=xPlin_x*mx;
% xElin_x=-6:0.3:0;
% xElin_y=xElin_x*mx;

figure(1);clf;
figset
plot(xP2d(1,:),xP2d(2,:),'-*k');
hold on
plot(xE2d(1,:),xE2d(2,:),'-Ok');
% hold on
% plot(xPlinCompare(1,:),xPlinCompare(2,:),'-.*k');
% hold on
% plot(xElinCompare(1,:),xElinCompare(2,:),'-.og');
axis([-6 2 -6 2])
xlabel('x-position (m)')
ylabel('y-position (m)')
legend('Pursuer trajectory','Evader trajectory');
figset

figset
figure(2);clf;
plottype=['k-x','k-*','k-0'];
plot(dt*(0:24),muHist(1,1:25),'k-x')
hold on
plot(dt*(0:24),muHist(2,1:25),'k-*')
hold on
if nmod>=3
plot(dt*(0:24),muHist(3,1:25),'k-o')
end
if nmod>=4
plot(dt*(0:24),muHist(4,1:25),'k-+')
end
if nmod>=5
plot(dt*(0:24),muHist(4,1:25),'k-s')
end
figset
xlabel('Time Elapsed (s)')
ylabel('Model Probability')
legend('VM/GT','GT','GT/PM','VM','other')
figset
%legend('Nash strategy','Non-Nash strategy') %update per side


