clear;clc;
% Main function for 2D game AI for pop-the-balloon

%
% Game state convention
%        +x
%         |
%         |
%   +y - -
%

%rngseedno=10;
rngseedno=40;
rng(rngseedno)

plotFlag=0;
plotEndFlag=1;

nPur=3;
nEva=3;
nTargets=2;

tstep=1;
tmax=20;


%load NN here
load nn1v1.mat
network1=net;
%1v1 for now
network2=[];


evaDynamics='f_dynEva';
purDynamics='f_dynPur';

[pairingsTemp, inactiveP, inactiveE] = simultaneousCombMatWithInactivev2(1:nPur,1:nEva);
pairings={};
[npair,~]=size(pairingsTemp);
for ij=1:npair
    rowtemp=pairingsTemp(ij,:);
    rowcull=rowtemp(rowtemp(:)~=0);
    for ik=1:length(rowcull)/2
        pairings{ij}{1}{ik}=rowcull(2*ik-1); 
        pairings{ij}{2}{ik}=rowcull(2*ik);
    end
end

%utemp=permn(-2:.5:2,2)';
%utemp=permn(-2:0.5:2,2)';
utemp=permn(-1:0.1:1,2)';
upmax=2;
umax=upmax;
utype.radius=upmax;
utemp=mapUtempToUvec(utemp,"circle",utype);

%Evader initial travel direction if unaware
traveldirEva=[1;0]; traveldirEva=traveldirEva/norm(traveldirEva);

Game.uType="velstep";
%Options: velstep (step of constant size, from velocity)
%         accel (double integrator)
Game.dt=tstep;

% Red and blue teams, Sred and Sblue
% S contains state information

QxPur = repmat(diag([100 100 0 0]),[1 1 nEva]); Rp = repmat(diag([0.1 1]),[1 1 nPur]);
QxEva = repmat(diag([100 50 0 0]),[1 1 nEva]); Re = repmat(diag([5 10]),[1 1 nEva]);

qrTrue=[diag(QxEva(:,:,1));diag(Re(:,:,1))];

eye2=eye(2); zer2=zeros(2,2);
Hpur=[eye2 zer2 zer2 zer2;
    zer2 zer2 eye2 zer2];
Heva=Hpur;
Rnoisepur=0.0002*eye(4);
Rnoiseeva=Rnoisepur;
H14=[Hpur zeros(4,6)];
VreserveP=100;
VreserveE=50;

dt=tstep;
n=2; %n is dim(x)/2 so I can copy/paste dynamic functions
Abk1=[eye(n) dt*eye(n); zeros(n,n) 0.3*eye(n)]; %Apur
Abk2=[eye(n) dt*eye(n); zeros(n,n) 0.3*eye(n)]; %Aeva
Aeva=blkdiag(Abk1,Abk2);
Gnoiseeva=blkdiag([eye(n)*dt^2/2;eye(n)*dt],[eye(n)*dt^2/2;eye(n)*dt]);
Qnoiseeva=0.002*eye(4);
Qnoisepur=Qnoiseeva;
GQG=Gnoiseeva*Qnoiseeva*Gnoiseeva';
cholQ2p=chol(Qnoisepur(1:2,1:2))'; cholQ2e=chol(Qnoisepur(3:4,3:4))';
Bnoiseeva=Gnoiseeva;
Peva=1e-2*eye(8);

Ppur=Peva;
%This can be shunted off to a separate function
n=0; %n is now the iteration index (yes, misuse of var names)
%xTrue=[[0 0 1.0 0.8]'; [10 4 0 0]'];
targetIndex={};
for ij=1:nPur
    xPur{ij}=[10*rand(2,1);0;0];
    Spur_p.fname{ij}=purDynamics;
    Spur_e.fname{ij}=purDynamics;
end
for ik=1:nEva
    xEva{ik}=[10*rand(2,1);0;0];
    Seva_p.fname{ik}=evaDynamics;
    Seva_e.fname{ik}=evaDynamics;
    targetIndex{ik}=floor(2*rand)+1;
end
targetLocation={};
targetValue={};
targetCost={};
for ih=1:nTargets
    targetLocation{ih}=10*rand(2,1);
    targetValue{ih}=50*rand;
    targetCost{ih}=50*rand;
end
maxDistToPenalize=5;
axisveck=[-5 15 -5 15];
xTrue{1,1}=xPur;
xTrue{1,2}=xEva;

gameStateValsEva.nx=4;
gameStateValsPur.nx=4;

if plotFlag==1
figure(1);clf
f1=scatter(xTrue(1),xTrue(2),'b');
hold on
f2=scatter(xTrue(5),xTrue(6),'r');
axis(axisveck)
end

% wStore{1}=wloc;
% xPartStore{1}=xPur_part;
% xPurS{1}=xPur;
% xEvaS{1}=xEva;
% xTrueS{1}=xTrue;
% dJS=[];
% Jp0=0;

% %initial dJ
% meann=zeros(14,1);
% wloc=1/npart*ones(npart,1);
% for ik=1:npart
%     meann=meann+wloc(ik)*xPur_part(:,ik);
% end
% dJ=meann(9:14)-qrTrue;
% dJS(:,1)=dJ;

for ij=1:tstep:tmax
    n=n+1
    tic
    
    %reset noise inflations
    QinflatePur=Qnoisepur;
    QinflateEva=Qnoiseeva;
    
    %Preload GT params used in one or more GT solvers
    gameState_p.xPur=xPur;
    gameState_p.xEva=xEva;
    gameState_p.dt=tstep;
    gameState_p.kMax=1;
    gameState_p.nu=2;
    gameState_p.numPursuers=nPur;
    gameState_p.numEvaders=nEva;
    gameState_p.inactiveSetP=inactiveP;
    gameState_p.inactiveSetE=inactiveE;
    gameState_p.targetIndexList=targetIndex;
    gameState_p.targetLocation=targetLocation;
    gameState_p.maxDistanceToTargetPenalized=maxDistToPenalize;
    gameState_p.nx=2;
%     uvelmatch=vmRGVO_max(xPur(1:4),xPur(5:8),upmax,2,tstep,uEvaBestPerformanceEstimate);
%     for ik=1:length(utemp)
%         Spur_p.uMat{ik}=utemp(:,ik);
%     end
    Spur_p.Jname='JswarmPur';
    Spur_p.Jparams.reserveRewardWeightPur=VreserveP;
    Spur_p.Jparams.QxPur=QxPur;
    Spur_p.Jparams.Rpur=Rp;
    Spur_p.pairings=pairings;
    Spur_p.Jparams.targetCost=targetCost;
    Seva_p.pairings=pairings;
    Seva_p.Jname='JswarmEva';
    Seva_p.Jparams.QxEva=QxEva;
    Seva_p.Jparams.Reva=Re;
    Seva_p.Jparams.Ropp=zeros(2,2);
    Seva_p.Jparams.targetValue=targetValue;
    Seva_p.Jparams.reserveRewardWeightEva=VreserveE;
    [upPairs_p,uePairs_e,flag,upMat,ueMat]=f_dynNN(Spur_p,Seva_p,gameState_p,zeros(4,1),network1,network2);
    if flag==1
        for iP=1:nPur
            uPur{iP}=upMat{upPairs_p}(:,iP);
        end
    elseif flag==0
        pair=randsample(1:length(upPairs_p),1,true,upPairs_p);
        for iP=1:nPur
            uPur{iP}=upMat{pair}(:,iP);
        end
    end
    
    %Omniscient evader
    gameState_e.xPur=xPur;
    gameState_e.xEva=xEva;
    gameState_e.dt=tstep;
    gameState_e.kMax=1;
    gameState_e.nu=2;
    Spur_e.Jname='JswarmPur';
    Spur_e.Jparams.targetCost=targetCost;
    Spur_e.Jparams.QxPur=QxPur;
    Spur_e.Jparams.Rpur=Rp;
    Spur_e.Jparams.Ropp=zeros(2,2);
    Spur_e.pairings=pairings;
    Seva_e.Jparams.targetValue=targetValue;
    Seva_e.Jname='JswarmEva';
    Spur_e.Jparams.reserveRewardWeightPur=VreserveP;
    Seva_e.Jparams.reserveRewardWeightEva=VreserveE;
    Seva_e.Jparams.QxEva=QxEva;
    Seva_e.pairings=pairings;
    Seva_e.Jparams.Reva=Re;
    Seva_e.Jparams.Ropp=zeros(2,2);
    gameState_e = gameState_p;
    [upPairs_e,uePairs_e,flag,upMat,ueMat]=f_dynNN(Spur_e,Seva_e,gameState_e,zeros(4,1),network1,network2);
    if flag==1
        for iE=1:nEva
            uEva{iE}=ueMat{uePairs_e}(:,nPur+iE);
        end
    elseif flag==0
        pair=randsample(1:length(uePairs_e),1,true,uePairs_e);
        for iE=1:nEva
            uEva{iE}=ueMat{pair}(:,nPur+iE);
        end
    end
    
    z21=zeros(2,1);
    for iP=1:nPur %Note: inactive players are shown [0] control
        xPur{iP}=feval(Spur_p.fname{iP},xPur{iP},uPur{iP},gameState_p.dt,z21);
    end
    for iE=1:nEva
        xEva{iE}=feval(Seva_e.fname{iE},xEva{iE},uEva{iE},gameState_e.dt,z21);
    end

    % Measurement
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
    xTrue{n+1,1}=xPur;
    xTrue{n+2,2}=xEva;
    xTrueS{n+1}=xTrue;
    
%     %cost calculation, need to debug error index length in J_pur
%     e=xTrue(1:4)-xTrue(5:8);
%     Jloc = e'*Spur_p.Jparams.Q*e + uPurTrue'*Spur_p.Jparams.Rself*uPurTrue + ...
%         uEvaTrue'*Spur_p.Jparams.Ropp*uEvaTrue;
%     Jp0=Jp0+Jloc;
    
    if plotFlag==1
    figure(1)
    pause(.1)
    delete(f1); delete(f2)
    f1=scatter(xTrue(1),xTrue(2),'b');
    hold on
    f2=scatter(xTrue(5),xTrue(6),'r');
    axis(axisveck)
    end
    
    tThisStep=toc
end

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
%     
%     xP=zeros(2,n+1);xE=zeros(2,n+1);
%     for ijk=1:n+1
%         xP(:,ijk)=xTrueS{ijk}(1:2); xE(:,ijk)=xTrueS{ijk}(5:6);
%     end
%     figure(3);clf;
%     plot(xP(1,:),xP(2,:),'-xr');
%     hold on
%     plot(xE(1,:),xE(2,:),'-ob');
%     title('Interceptor using velocity matching vs unaware evader');
%     xlabel('East displacement (m)');
%     ylabel('North displacement (m)');
%     legend('Pursuer','Evader');
%     axis([0 90 0 10])
%     figset
    
end
