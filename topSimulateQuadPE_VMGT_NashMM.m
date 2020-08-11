clear;clc;
beep off;

rngseedno=457;
rng(rngseedno);

%NOTE: TUNING IN VMRGO REQUIRES USING fdynEva AND fdynPur FOR PM SIM
%  SET DRAG PARAMS IN BOTH TO 0.
%NOTE: set the target function to generateCostMatricesVMquad in f_dyn2
%  BEFORE running this; hardcoded the other way to remain backwards
%  compatible


% standard controller:
% runtime: 5.0165e+03
% Jp: 1.0120e+04
% Je: 9.6958e+03

% VMGT:
% runtime: 476.6351
% Jp: 1.0408e+04
% Je: 9.3878e+03

% VMGT w/NN
% runtime: 1.7013e+03
% Jp: 1.0318e+04
% Je: 9.4986e+03

% VMGT BADDIAG
% JJp =
%    1.0016e+04
% JJe =
%    9.0306e+03

% standard BADDIAG
% JJp =
%    1.4131e+04
% JJe =
%    5.9543e+03

% General control type flags
useGameTheoreticController=true;
usePureVelMatchController=false;
scaleVec=0.8; %magnitude of desired uE control relative to uP control
vmtune=0.8; %deceleration parameter for VM

% Control type flags, if GT specified
% select from: vmquad, gt_overx
Spur.controlType='gt_overx';
Seva.controlType='gt_overx';
% gameState_p.controlType='gt_overx';
omega_hover=4.95;
nmod=3;

% load nnTrainSets\nnQuadDyn\network.mat
% gameState_p.NN=net;
gameState.tryNN=false;

umax=.5;
uLmax=8; %max low-level control

dt=0.1;
t0=0;
tmax=5;

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

Qpur=zeros(12,12);
Qpur(7:9,7:9)=diag([5,5,0]);
Qeva=zeros(12,12);
Qeva(7:8,7:8)=5*eye(2);
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
    
    %     upurset=umin:du:umax;
    %     %uPur=combvec(upurset,upurset,upurset,upurset);
    %     uPur=combvec(upurset,upurset,upurset);
    %     uEva=uPur;
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
    
    if useGameTheoreticController
        % Guessing pursuer
        gameState.xPur=xPur(1:12);
        gameState.xEva=xPur(13:24);
        gameState.dt=dt;
        gameState.kMax=1;
        gameState.nu=2;
        gameState.discType='overX';
        gameState.uMaxP=umax;
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
        Spur.Jname='J_purQuad';
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
        Seva.Jname='J_evaQuad';
        Seva.fname='f_dynEvaQuad';
        Seva.UseVelMatch=true;
        % Propagate to next time step
        [up,ue,flag,uPSampled,uESampled,Sminimax,Smisc]=f_dyn2(Spur,Seva,gameState,zeros(4,1));
        uPurTrue=uPSampled;
        uEvaTrue=uESampled;
        uEvaNash=uESampled;
        

        SInput.Spur=Spur; SInput.Seva=Seva;
        SInput.Seva.uMat={}; SInput.Seva.uMat{1}=uEvaTrue; SInput.Seva.controlType='gt_overu';
        SInput.utemp=utempFine; SInput.uvec=uvecFine; SInput.umax=umax;
        SInput.gameState=gameState;
        SInput.player='pur'; SInput.type='discretize';
        u2=optimizeGivenEnemyControl(SInput);
        
    elseif usePureVelMatchController
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
    
%     uEvaTrue=uEvaTrue+[.2;.2;-.2;-.2];
    
    uPhist=[uPhist uPurTrue];
    uEhist=[uEhist uEvaTrue];
    xTrue(1:12)=f_dynPurQuad(xTrue(1:12),uPurTrue(:,1),dt,zeros(2,1));
    xTrue(13:24)=f_dynEvaQuad(xTrue(13:24),uEvaTrue(:,1),dt,zeros(2,1));
    xD=[zeros(24,1) xTrue];
    
    Jpur=feval(Spur.Jname,xD(1:12,:),xD(13:24,:),uPurTrue,uEvaTrue,Spur.Jparams);
    Jeva=feval(Seva.Jname,xD(13:24,:),xD(1:12,:),uEvaTrue,uPurTrue,Seva.Jparams);
    JJp=JJp+Jpur
    JJe=JJe+Jeva
    
    lambdaTemp=-1*ones(nmod,1);
    for ij=1:nmod
        uEvaTemp=[];
        Ru=[];
        if ij==1
            uEvaTemp=uEvaNash;
            Ru=0.01*du*eye(4);
        elseif ij==2
            uEvaTemp=omega_hover;
            Ru=1*eye(4);
        elseif ij==3
            loadPointMassControlParams;
            Ru=0.01*du*eye(4);
        elseif ij==4
            uEvaTemp=Sminimax.uE;
            Ru=0.01*eye(4);
        end
        [xhatEp1,Pp1]=ukfPropagate(xhatE(:,ij),PhatE(:,:,ij),uEvaTemp,Ru,dt,'f_dynEvaQuad');
        
        zMeas=xTrue(13:24)+chol(Rk)*rand(12,1);
        [xhatE(:,ij),PhatE(:,:,ij),nu,Sk,Wk]=kfMeasure(xhatEp1,Pp1,zMeas,eye(length(ewxvEva)),Rk);
        
        Skt=triu(Sk); Skt=Skt'+Skt; Skt(1:13:end)=diag(Sk); %forces symmetry
        normpEval= mvnpdf(nu,zeros(length(ewxvEva),1),Skt);
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
plot(xP2d(1,:),xP2d(2,:),'-*b');
hold on
plot(xE2d(1,:),xE2d(2,:),'-Or');
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
plot(dt*(0:24),muHist(4,1:25),'k-0')
end
figset
xlabel('Time Elapsed (s)')
ylabel('Model Probability')
%legend('Nash strategy','Non-Nash strategy') %update per side


