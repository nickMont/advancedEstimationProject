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

tstep=1;
tmax=20;

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

qrTrue=[diag(QxEva);diag(Re)];

eye2=eye(2); zer2=zeros(2,2);
Hpur=[eye2 zer2 zer2 zer2;
    zer2 zer2 eye2 zer2];
Heva=Hpur;
Rnoisepur=0.0002*eye(4);
Rnoiseeva=Rnoisepur;
H14=[Hpur zeros(4,6)];

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
xTrue=[[0 0 1.0 0.8]'; [10 4 0 0]'];
xPur=xTrue;
xEva=xTrue;
axisveck=[-20 20 -5 35];

gameStateValsEva.nx=4;
gameStateValsPur.nx=4;

if plotFlag==1
figure(1);clf
f1=scatter(xTrue(1),xTrue(2),'b');
hold on
f2=scatter(xTrue(5),xTrue(6),'r');
axis(axisveck)
end

wStore{1}=wloc;
xPartStore{1}=xPur_part;
xPurS{1}=xPur;
xEvaS{1}=xEva;
xTrueS{1}=xTrue;
dJS=[];
Jp0=0;

%initial dJ
meann=zeros(14,1);
wloc=1/npart*ones(npart,1);
for ik=1:npart
    meann=meann+wloc(ik)*xPur_part(:,ik);
end
dJ=meann(9:14)-qrTrue;
dJS(:,1)=dJ;

for ij=1:tstep:tmax
    n=n+1
    tic
    
    %reset noise inflations
    QinflatePur=Qnoisepur;
    QinflateEva=Qnoiseeva;
    
    xPurMean=[xPur;diag(QxEva);diag(Re)];
    
    %Preload GT params used in one or more GT solvers
    gameState_p.xPur=xPur;
    gameState_p.xEva=xEva;
    gameState_p.dt=tstep;
    gameState_p.kMax=1;
    gameState_p.nu=2;
    gameState_p.numPursuers=nPur;
    gameState_p.numEvaders=nEva;
%     uvelmatch=vmRGVO_max(xPur(1:4),xPur(5:8),upmax,2,tstep,uEvaBestPerformanceEstimate);
%     for ik=1:length(utemp)
%         Spur_p.uMat{ik}=utemp(:,ik);
%     end
    Spur_p.Jname='JswarmPur';
    Spur_p.fname='f_dynPur';
    Spur_p.Jparams.Q=QxPur;
    Spur_p.Jparams.Rself=Rp;
    Seva_p.Jname='JswarmEva';
    Seva_p.fname='f_dynEva';
    Seva_p.Jparams.QxPur=QxPur;
    Seva_p.Jparams.Rself=Re;
    Seva_p.Jparams.Ropp=zeros(2,2);
    [upPairs_p,uePairs_e,flag,upMat,ueMat]=f_dynNN(Spur_p,Seva_p,gameState_p,zeros(4,1));
    for iP=1:nPur
        uPur{iP}=upMat{uePairs_p}{iP};
    end
    
    %Omniscient evader
    gameState_e.xPur=xPur;
    gameState_e.xEva=xEva;
    gameState_e.dt=tstep;
    gameState_e.kMax=1;
    gameState_e.nu=2;
    Spur_e.Jname='JswarmPur';
    Spur_e.fname='f_dynPur';
    Spur_e.Jparams.Q=QxPur;
    Spur_e.Jparams.Rself=Re;
    Spur_e.Jparams.Ropp=zeros(2,2);
    Seva_e.uMat = Spur_e.uMat;
    Seva_e.Jname='JswarmEva';
    Seva_e.fname='f_dynEva';
    Seva_e.Jparams.Q=QxEva;
    Seva_e.Jparams.Rself=Rp;
    Seva_e.Jparams.Ropp=zeros(2,2);
    gameState_e = gameState_p;
    [upPairs_e,uePairs_e,flag,upMat,ueMat]=f_dynNN(Spur_e,Seva_e,gameState_e,zeros(4,1));
    for iE=1:nEva
        uEva{iE}=ueMat{uePairs_e}{iE};
    end
    
    for iP=1:nPur
        activeP=upPairs_p{iE};
        if ~ismember(iP,activP(2:2:end))
            xPur{iP}=feval(Spur_p.fname,xPur{iP},uPur{iP});
        end
    end
    for iE=1:nEva
        activeE=uePairs{iE};
        if ~ismember(iE,activE(2:2:end))
            xEva{iE}=feval(Seva_e.fname,xEva{iE},uEva{iE});
        end
    end

    % Measurement
    zPur=Hpur*xTrue+chol(Rnoisepur)'*randn(4,1);
    zEva=Heva*xTrue+chol(Rnoiseeva)'*randn(4,1);
    
    if pursuerUsesGT || pursuerUsesVelmatch
        [xPur,Ppur]=kfstep(xPur,zPur,Aeva,Bnoiseeva,[uPurTrue;uEvaEst],Gnoiseeva,QinflatePur,Ppur,Heva,Rnoiseeva);
        %NOTE: PROPAGATE FILTER HERE
    end
    if evaderUsesGT
        [xEva,Peva]=kfstep(xEva,zEva,Aeva,Bnoiseeva,[uPurEst;uEvaTrue],Gnoiseeva,QinflateEva,Peva,Heva,Rnoisepur);
    end
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
