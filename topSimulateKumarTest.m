clear;clc;loadenv;


% TO DO: 2x COMPUTE FOR P/E SPLIT


% Game state convention
%        +x
%         |
%         |
%   +y - -
%

%rngseedno=10;
rngseedno=42;
rng(rngseedno)

plotFlag=0;
plotEndFlag=0;
%npart=5000;
npart=500;

%Evader control type info
evaderUsesGT=1;
evaderUsesKumar=~evaderUsesGT;

%Pursuer control type info
pursuerUsesGT=evaderUsesGT;
pursuerUsesKumar=~pursuerUsesGT;

% Try discrete linear propagation instead of ode45
gameStateVals.tryLinearPropagation = false;

%Will velmatching use a control estimate?
%uEvaBestPerformanceEstimate=[0.9;0]; %lower can produce oscillations
uEvaBestPerformanceEstimate=[0;0]; 

cd = 0.01; %drag coefficient, always positive

tstep=1;
tmax=10;
dt = tstep;
tplan = tmax;
kmax = floor(tplan/tstep);
kmax = 1;

%utemp=permn(-2:.5:2,2)';
% rSet = -(0:0.2:1);
% tSet = 0:0.25:0.5; %multiplied by pi in mapUtempToUvec
rSet = (0 : 0.25 : 1);
tSet = (0 : 1/4 : 2); %multiplied by pi in mapUtempToUvec
umax=0.5;
umaxP=umax;
umaxE=umax;
if pursuerUsesGT || evaderUsesGT
    utemp = allcomb(rSet,tSet)';
    utype.radius=umax;
    utemp=mapUtempToUvec(utemp,"circle",utype);
    max_steps_to_predict=kmax;
    utemp_perm = permuteOverTime(utemp,max_steps_to_predict);
end

%Evader initial travel direction if unaware
traveldirEva=[1;0]; traveldirEva=traveldirEva/norm(traveldirEva);

Game.uType="velstep";
%Options: velstep (step of constant size, from velocity)
%         accel (double integrator)
Game.dt=tstep;

% Initial state
xTrue=[1 1 0 0]';

Qpur = diag([10 10 10 10]); Rpur = diag([10 10]);
Qeva = diag([10 10 0 0]); Reva = diag([50 50]);

qrTrue=[diag(Qeva);diag(Reva)];

eye2=eye(2); zer2=zeros(2,2);
Hpur=eye(4);
Heva=Hpur;
Rnoisepur=0.0002*eye(4);
Rnoiseeva=Rnoisepur;
H14=[Hpur zeros(4,6)];

n=2; %n is dim(x)/2 so I can copy/paste dynamic functions
% Abk1=[eye(n) dt*eye(n); zeros(n,n) 0.3*eye(n)]; %Apur
% Abk2=Abk1;
% Aeva=blkdiag(Abk1,Abk2);
Gnoiseeva=blkdiag([eye(n)*dt^2/2;eye(n)*dt],[eye(n)*dt^2/2;eye(n)*dt]);
Qnoiseeva=0.002*eye(4);
Qnoisepur=Qnoiseeva;
GQG=Gnoiseeva*Qnoiseeva*Gnoiseeva';
cholQ2p=chol(Qnoisepur(1:2,1:2))'; cholQ2e=chol(Qnoisepur(3:4,3:4))';
Bnoiseeva=Gnoiseeva;
Peva=1e-2*eye(8);

usesKumar=false;
if evaderUsesKumar||pursuerUsesKumar
    usesKumar=true;
end
Gcontinuous1=[0 0
              0 0
              1 0
              0 1];
Gcontinuous2=[0 0
              0 0
              1 0
              0 1];          
Fcontinuous=[0 0 1 0
             0 0 0 1
             0 0 -cd 0
             0 0 0 -cd];
nx = length(Fcontinuous);

%calculate state transition matrix here
A=expm(Fcontinuous*dt);
Bstack2 = ...
    [dt^2/2 0 -dt^2/2 0
    0 dt^2/2 0 -dt^2/2
    dt 0 -dt 0
    0 dt 0 -dt];
%Note: B,u continuous over time step, so zero-state response is just
%  the (integral of the state transition matrix)*B*u
B1 = (expm(Fcontinuous*dt)-eye(4))*inv(A)*Gcontinuous1;
B2 = (expm(Fcontinuous*dt)-eye(4))*inv(A)*Gcontinuous2;
Bstack1 = [B1 -B2];

BstackUse = Bstack2;

Ppur=Peva;
%This can be shunted off to a separate function
n=0; %n is now the iteration index (yes, misuse of var names)
xPur=xTrue;
xEva=xTrue;
axisveck=[-20 20 -5 35];


gameStateVals.nX=4;
gameStateVals.R21=zeros(2,2);
gameStateVals.R12=zeros(2,2);
gameStateVals.F=Fcontinuous;
gameStateVals.G1=Gcontinuous1;
gameStateVals.G2=Gcontinuous2;
gameStateVals.Q1=Qpur;
gameStateVals.Q2=Qeva;
gameStateVals.W=Qnoiseeva;
gameStateVals.V1=Ppur(1:4,1:4);
gameStateVals.V2=Peva(1:4,1:4);
gameStateVals.R22=Reva;
gameStateVals.R11=Rpur;
gameStateVals.H1=eye(4);
gameStateVals.H2=eye(4);
gameStateVals.umax=umax;
gameStateVals.A = A;
gameStateVals.Bstack = BstackUse;
Pvec0Pur=repmat(reshape(Qnoiseeva+Qnoisepur,[16,1]),[3 2]);
Qvec0Pur= [[reshape(Qpur, [16 1]); reshape(Qeva, [16,1]); zeros(16,1); zeros(16,1)] [reshape(Qpur, [16 1]); reshape(Qeva, [16,1]); zeros(16,1); zeros(16,1)]];

tvec0pPur=[0 tplan]; tvec0pEva=[0 tplan];
tvec0qPur=[tplan 0]; tvec0qEva=[tplan 0];
if usesKumar
for ik=1:3 %fwd/bwd pass three times
    propagateP_Pur=@(t,x) odeKumarP(t,x,Qvec0Pur,tvec0qPur,gameStateVals);
    [tvec0pPur,Pvec0Pur]=ode45(propagateP_Pur,[0 tplan],Pvec0Pur(:,1));
    Pvec0Pur=Pvec0Pur';
    
    propagateQ_Pur=@(t,x) odeKumarQ(t,x,Pvec0Pur,tvec0pPur,gameStateVals);
    [tvec0qPur,Qvec0Pur]=ode45(propagateQ_Pur,[tplan 0],Qvec0Pur(:,1));
    Qvec0Pur=Qvec0Pur';
    %NOTE: CHECK THAT OUTPUTS ARE IN THE RIGHT TIME SERIES ORDER
end
end

% Pvec0Pur
% Qvec0Pur

if plotFlag==1
figure(1);clf
f1=scatter(xTrue(1),xTrue(2),'b');
hold on
f2=scatter(xTrue(5),xTrue(6),'r');
axis(axisveck)
end

xTrueS{1}=xTrue;
dJS=[];
Jp0=0;

zPur=Hpur*xTrue+chol(Rnoisepur)'*randn(4,1);
zEva=Heva*xTrue+chol(Rnoiseeva)'*randn(4,1);

tic

% Plan trajectory
if pursuerUsesGT || evaderUsesGT
    gameState.params=gameStateVals;
    gameState.x0 = zPur;
    gameState.t0 = 0;
    gameState.dt = tstep;
    gameState.kmax = kmax;
    [~,~,nUU]=size(utemp_perm);
    Spur.uMat=cell(nUU,1);
    for ij=1:nUU
        Spur.uMat{ij,1}=utemp_perm(:,:,ij);
    end
    Seva.uMat=Spur.uMat;
    [uPurM,uEvaM,outflag,uVP,uVE] = f_dynTopKumar(Spur,Seva,gameState,[],[]);
    indInController = 0;
end
needToReplan = false;


tStack=[];
xStack=[];
% Evaluate trajectory
u1Stack=[];
u2Stack=[];
for tind = 0:tstep:tmax-tstep
    if needToReplan
        gameState.x0 = zPur;
        indInController = 0;
        [uPurM,uEvaM,outflag,uVP,uVE] = f_dynTopKumar(Spur,Seva,gameState,[],[]);
    end
    if pursuerUsesGT || evaderUsesGT
        indInController = indInController+1;
        if indInController >= gameState.kmax
            needToReplan=true;
        else
            needToReplan=false;
        end
    end
    if pursuerUsesGT
        uPurTrue = uVP(:,indInController);
    end
    if evaderUsesGT
        uEvaTrue = uVE(:,indInController);
    end
    
    if pursuerUsesKumar || evaderUsesKumar
        [~,tqi] = min(abs(tvec0qPur-tind));
        [~,tpi] = min(abs(tvec0pPur-tind));
        Qv = Qvec0Pur(:,tqi);
        Pv = Pvec0Pur(:,tpi);
        [Q1,Q2,Q3,Q4] = QvecToMats(Qv,nx);
        [P1,P2,P3] = PvecToMats(Pv,nx);
    end
    if pursuerUsesKumar
        uPurTrue = -(gameStateVals.R11)*gameStateVals.G1'*(Q1*zPur +...
            Q3*(zPur-zEva));
        uN = norm(uPurTrue);
        uN = saturationF(uN,0,umaxP);
        if uN>1e-10
            uPurTrue = uN*uPurTrue/norm(uPurTrue);
        end
    end
    if evaderUsesKumar
        uEvaTrue = -inv(gameStateVals.R22)*gameStateVals.G2'*Q2*zEva;
        uN = norm(uEvaTrue);
        uN = saturationF(uN,0,umaxE);
        if uN>1e-10
            uEvaTrue = uN*uEvaTrue/norm(uEvaTrue);
        end
    end
    
%     dd = atan2(uPurTrue(2),uPurTrue(1))
    
    uPurApplied = uPurTrue
    uEvaApplied = uEvaTrue
    
    paramsSim = gameStateVals;
    paramsSim.u1 = uPurTrue;
    paramsSim.u2 = uEvaTrue;
    os = @(t,x) fdyn_kumarSamp(t,x,paramsSim);
    [t2,x2]=ode45(os,[tind tind+tstep],xTrue);
    
    tStack = [tStack;t2];
    xStack = [xStack;x2];
    u1Stack = [u1Stack repmat(uPurTrue, [1 length(t2)])];
    u2Stack = [u2Stack repmat(uEvaTrue, [1 length(t2)])];
    
    xTrue = (x2(end,:))';
    zPur=Hpur*xTrue+chol(Rnoisepur)'*randn(4,1);
    zEva=Heva*xTrue+chol(Rnoiseeva)'*randn(4,1);
end

[J1,J1run,J1final] = evalCostFunc(tStack,xStack,u1Stack,u2Stack,'j1Int','j1Final',paramsSim);
[J2,J2run,J2final] = evalCostFunc(tStack,xStack,u1Stack,u2Stack,'j2Int','j2Final',paramsSim);
J2=-J2; J2run=-J2run; J2final=-J2final;

toc

J1f=J1
J2f=J2


% for ij=0:tstep:tmax
%     n=n+1
%     tic
%     
%     %reset noise inflations
%     QinflatePur=Qnoisepur;
%     QinflateEva=Qnoiseeva;
%     
%     xPurMean=[xPur;diag(Qeva);diag(Reva)];
%     
%     %Preload GT params used in one or more GT solvers
%     gameState_p.xPur=xPurMean(1:4);
%     gameState_p.xEva=xPurMean(5:8);
%     gameState_p.dt=tstep;
%     gameState_p.kMax=max_steps_to_predict;
%     gameState_p.nu=2;
%     uvelmatch=vmRGVO_max(xPur(1:4),xPur(5:8),upmax,2,tstep,uEvaBestPerformanceEstimate);
%     Spur_p.uMat={0}; Seva_p.uMat={0};
%     for ik=1:max(size(utemp_perm))
%         Spur_p.uMat{ik}=squeeze(utemp_perm(:,:,ik));
%     end
%     Spur_p.Jname='J_pur';
%     Spur_p.fname='f_dynPur';
%     Spur_p.Jparams.Q=Qpur;
%     Spur_p.Jparams.Rself=Rpur;
%     Spur_p.Jparams.Ropp=zeros(2,2);
%     Seva_p.uMat = Spur_p.uMat;
%     Spur_p.uMat{length(utemp)+1}=uvelmatch;
%     Seva_p.Jname='J_eva';
%     Seva_p.fname='f_dynEva';
%     Seva_p.Jparams.Q=diag(xPurMean(9:12));
%     Seva_p.Jparams.Rself=diag(xPurMean(13:14));
%     Seva_p.Jparams.Ropp=zeros(2,2);
%     
%     if pursuerUsesVelmatch
%         uPurTrue=uvelmatch;
%         uEvaEst=zeros(2,1);
%     elseif pursuerUsesGT
%         if useNNforGT
%             qqrr=0.1*[diag(Qpur); diag(Qeva); diag(Rpur); diag(Reva)];
%             ministate=xPurMean;
%             uStack=predict(network1,[ministate;qqrr]);
%             uPurTrue=uStack(1:2);
%             uEvaEst=uStack(3:4);
%         else
%             % Guessing pursuer
%             gameState_p.xPur=xPurMean(1:4);
%             gameState_p.xEva=xPurMean(5:8);
%             gameState_p.dt=tstep;
%             gameState_p.kMax=1;
%             gameState_p.nu=2;
%             Spur_p.uMat={0}; Seva_p.uMat={0};
%             for ik=1:length(utemp)
%                 Spur_p.uMat{ik}=utemp(:,ik);
%             end
%             Spur_p.Jname='evalCostFunc';
%             Spur_p.fname='f_dynPur';
%             Spur_p.Jparams.Q=Qpur;
%             Spur_p.Jparams.Rself=Rpur;
%             Spur_p.Jparams.Ropp=zeros(2,2);
%             Seva_p.uMat = Spur_p.uMat;
%             Spur_p.uMat{length(utemp)+1}=uvelmatch;
%             Seva_p.Jname='evalCostFunc';
%             Seva_p.fname='f_dynEva';
%             Seva_p.Jparams.Q=diag(xPurMean(9:12));
%             Seva_p.Jparams.Rself=diag(xPurMean(13:14));
%             Seva_p.Jparams.Ropp=zeros(2,2);
%             % Propagate to next time step
%             [up,ue,flag]=f_dyn(Spur_p,Seva_p,gameState_p,zeros(4,1));
%             if flag==0
%                 upp=randsample(1:length(Spur_p.uMat),1,true,up);
%                 uPurTrue=Spur_p.uMat{upp}(:,1);
%                 uEvaEst=zeros(gameState_p.nu,gameState_p.kMax);
%                 for ik=1:length(Seva_p.uMat)
%                     uEvaEst=uEvaEst+ue(ik)*Seva_p.uMat{ik}(:,1);
%                 end
%                 QinflatePur=Qnoisepur+blkdiag(zer2,1*eye2);
%             else
%                 uPurTrue=up;
%                 uEvaEst=ue;
%             end
%         end
%     end
%     if pursuerUsesGT || pursuerUsesVelmatch
%         xPur_bar(1:4)=f_dynPur(xPurMean(1:4),uPurTrue,tstep,zeros(2,1));
%         xPur_bar(5:8)=f_dynEva(xPurMean(5:8),uEvaEst ,tstep,zeros(2,1));
%     end
%     
%     if evaderUsesGT
%         %Omniscient evader
%         gameState_e.xPur=xEva(1:4);
%         gameState_e.xEva=xEva(5:8);
%         gameState_e.dt=tstep;
%         gameState_e.kMax=max_steps_to_predict;
%         gameState_e.nu=2;
%         Spur_e.uMat={0}; Seva_e.uMat={0};
%         for ik=1:max(size(utemp_perm))
%             Spur_e.uMat{ik}=squeeze(utemp_perm(:,:,ik));
%         end
%         Spur_e.Jname='J_pur';
%         Spur_e.fname='f_dynPur';
%         Spur_e.Jparams.Q=Qpur;
%         Spur_e.Jparams.Rself=Rpur;
%         Spur_e.Jparams.Ropp=zeros(2,2);
%         Seva_e.uMat = Spur_e.uMat;
%         Seva_e.Jname='J_eva';
%         Seva_e.fname='f_dynEva';
%         Seva_e.Jparams.Q=Qeva;
%         Seva_e.Jparams.Rself=Reva;
%         Seva_e.Jparams.Ropp=zeros(2,2);
%         gameState_e = gameState_p;
%         gameState_e.xPur=xEva(1:4);
%         gameState_e.xEva=xEva(5:8);
%         if evaderIsOblivious
%             uPurEst=zeros(2,1);
%             uEvaTrue=upmax/3*traveldirEva;
%         else
%             [up,ue,flag]=f_dyn(Spur_e,Seva_e,gameState_e,zeros(4,1));
%             if flag==0
%                 uee=randsample(1:length(Seva_e.uMat),1,true,ue);
%                 uEvaTrue=Seva_e.uMat{uee}(:,1);
%                 uPurEst=zeros(gameState_p.nu,gameState_p.kMax);
%                 for ik=1:length(Spur_e.uMat)
%                     uPurEst=uPurEst+up(ik)*Spur_e.uMat{ik}(:,1);
%                 end
%                 QinflateEva=Qnoiseeva+blkdiag(1*eye2, zer2);
%             else
%                 uEvaTrue=ue;
%                 uPurEst=up;
%             end
%         end
%     end
%     
%     if pursuerUsesKumar
%         uPurTrue = -
%         
%     end
%     
%     if evaderUsesKumar
%         
%     end
%     
% 
%     % Measurement
%     zPur=Hpur*xTrue+chol(Rnoisepur)'*randn(4,1);
%     zEva=Heva*xTrue+chol(Rnoiseeva)'*randn(4,1);
%     
%     if pursuerUsesGT || pursuerUsesVelmatch
%         [xPur,Ppur]=kfstep(xPur,zPur,Aeva,Bnoiseeva,[uPurTrue;uEvaEst],Gnoiseeva,QinflatePur,Ppur,Heva,Rnoiseeva);
%         %NOTE: PROPAGATE FILTER HERE
%     end
%     if evaderUsesGT
%         [xEva,Peva]=kfstep(xEva,zEva,Aeva,Bnoiseeva,[uPurEst;uEvaTrue],Gnoiseeva,QinflateEva,Peva,Heva,Rnoisepur);
%     end
%     xTrueS{n+1}=xTrue;
%     
% % %     %cost calculation, need to debug error index length in J_pur
% %     e=xTrue(1:4)-xTrue(5:8);
% %     Jloc = e'*Spur_p.Jparams.Q*e + uPurTrue(:,1)'*Spur_p.Jparams.Rself*uPurTrue(:,1) + ...
% %         uEvaTrue(:,1)'*Spur_p.Jparams.Ropp*uEvaTrue(:,1);
% %     Jp0=Jp0+Jloc;
%     
%     if plotFlag==1
%     figure(1)
%     pause(.1)
%     delete(f1); delete(f2)
%     f1=scatter(xTrue(1),xTrue(2),'b');
%     hold on
%     f2=scatter(xTrue(5),xTrue(6),'r');
%     axis(axisveck)
%     end
%     
%     tThisStep=toc
% end

if plotEndFlag==1
%     figure(2);clf;
%     subplot(3,1,1);
%     plot(1:n+1,dJS(1,:),'-.r');
%     hold on
%     plot(1:n+1,dJS(2,:),'-ob');
%     legend('\DeltaQ_{xx}','\DeltaQ_{yy}')
%     xlabel('Time (s)');
%     ylabel('Cost parameter (m^{-2})');
%     figset
%     
%     subplot(3,1,2);
%     plot(1:n+1,dJS(3,:),'-.r');
%     hold on
%     plot(1:n+1,dJS(4,:),'-ob');
%     legend('\DeltaQ_{vx}','\DeltaQ_{vy}');
%     xlabel('Time (s)');
%     ylabel('Cost parameter (m^{-2}s^2)');
%     figset
%     
%     subplot(3,1,3);
%     plot(1:n+1,dJS(5,:),'-.r');
%     hold on
%     plot(1:n+1,dJS(6,:),'-ob');
%     legend('\DeltaR_{x}','\DeltaR_{y}');
%     xlabel('Time (s)');
%     ylabel('Cost parameter (m^{-2}s^4)');
%     figset
    
    xP=zeros(2,n+1);xE=zeros(2,n+1);
    for ijk=1:n+1
        xP(:,ijk)=xTrueS{ijk}(1:2); xE(:,ijk)=xTrueS{ijk}(5:6);
    end
    figure(3);clf;
    plot(xP(1,:),xP(2,:),'-xr');
    hold on
    plot(xE(1,:),xE(2,:),'-ob');
    title('Interceptor using velocity matching vs unaware evader');
    xlabel('East displacement (m)');
    ylabel('North displacement (m)');
    legend('Pursuer','Evader');
    axis([0 90 0 10])
    figset
    
end
