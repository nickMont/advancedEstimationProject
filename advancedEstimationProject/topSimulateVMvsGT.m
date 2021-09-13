clear;clc;
% Main function for 2D game

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

numEvaders=3;
numPursuers=3;


plotFlag=0;
plotEndFlag=1;
%npart=5000;
npart=500;

evaderIsOblivious=false;
pursuerUsesVelmatch=false;
%Will velmatching use a control estimate?
%uEvaBestPerformanceEstimate=[0.9;0]; %lower can produce oscillations
uEvaBestPerformanceEstimate=[0;0]; 

tstep=1;
tmax=20;

%utemp=permn(-2:.5:2,2)';
%utemp=permn(-2:0.5:2,2)';
utemp=permn(-1:0.1:1,2)';
upmax=2;
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

Qpur = diag([100 100 0 0]); Rpur = diag([0.1 1]);
Qeva = diag([100 50 0 0]); Reva = diag([5 10]);

qrTrue=[diag(Qeva);diag(Reva)];

eye2=eye(2); zer2=zeros(2,2);
Hpur=[eye2 zer2 zer2 zer2;
    zer2 zer2 eye2 zer2];
Heva=Hpur;
Rnoisepur=0.0002*eye(4);
Rnoiseeva=Rnoisepur;
H14=[Hpur zeros(4,6)];

dt=tstep;
n=2; %n is dim(x)/2 so I can copy/paste dynamic functions
Abk1=[eye(n) dt*eye(n); zeros(n,n) 0.6*eye(n)]; %Apur
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

if plotFlag==1
figure(1);clf
f1=scatter(xTrue(1),xTrue(2),'b');
hold on
f2=scatter(xTrue(5),xTrue(6),'r');
axis(axisveck)
end

% Scaling on excitation for Jparams
cholQexcite=0.05;
cholRexcite=0.05;
% P to do thing
cholExcitePlusOne=0.05;
cholExciteDropOrder=0.00;
orderExcite=0.5;

% wStore{1}=wloc;
% xPartStore{1}=xPur_part;
xPurS{1}=xPur;
xEvaS{1}=xEva;
xTrueS{1}=xTrue;

evaderVec=1:numEvaders;
pursuerVec=1:numPursuers;
evaderVec=[evaderVec 0];
pursuerVec=[pursuerVec 0];
Jp0=0;
Je0=0;



for ij=1:tstep:tmax
    n=n+1
    tic
    
    %reset noise inflations
    QinflatePur=Qnoisepur;
    QinflateEva=Qnoiseeva;
    
    xPurMean=[xPur;diag(Qeva);diag(Reva)];
    
    %Preload GT params used in one or more GT solvers
    gameState_p.xPur=xPurMean(1:4);
    gameState_p.xEva=xPurMean(5:8);
    gameState_p.dt=tstep;
    gameState_p.kMax=1;
    gameState_p.nu=2;
    uvelmatch=vmRGVO_max(xPur(1:4),xPur(5:8),upmax,2,tstep,uEvaBestPerformanceEstimate);
    Spur_p.uMat={0}; Seva_p.uMat={0};
    for ik=1:length(utemp)
        Spur_p.uMat{ik}=utemp(:,ik);
    end
    Spur_p.Jname='J_pur';
    Spur_p.fname='f_dynPur';
    Spur_p.Jparams.Q=Qpur;
    Spur_p.Jparams.Rself=Rpur;
    Spur_p.Jparams.Ropp=zeros(2,2);
    Seva_p.uMat = Spur_p.uMat;
    Spur_p.uMat{length(utemp)+1}=uvelmatch;
    Seva_p.Jname='J_eva';
    Seva_p.fname='f_dynEva';
    Seva_p.Jparams.Q=diag(xPurMean(9:12));
    Seva_p.Jparams.Rself=diag(xPurMean(13:14));
    Seva_p.Jparams.Ropp=zeros(2,2);
    
    if pursuerUsesVelmatch
        uPurTrue=uvelmatch;
        uEvaEst=zeros(2,1);
    else
        % Guessing pursuer
        gameState_p.xPur=xPurMean(1:4);
        gameState_p.xEva=xPurMean(5:8);
        gameState_p.dt=tstep;
        gameState_p.kMax=1;
        gameState_p.nu=2;
        Spur_p.uMat={0}; Seva_p.uMat={0};
        for ik=1:length(utemp)
            Spur_p.uMat{ik}=utemp(:,ik);
        end
        Spur_p.Jname='J_pur';
        Spur_p.fname='f_dynPur';
        Spur_p.Jparams.Q=Qpur;
        Spur_p.Jparams.Rself=Rpur;
        Spur_p.Jparams.Ropp=zeros(2,2);
        Seva_p.uMat = Spur_p.uMat;
        Spur_p.uMat{length(utemp)+1}=uvelmatch;
        Seva_p.Jname='J_eva';
        Seva_p.fname='f_dynEva';
        Seva_p.Jparams.Q=diag(xPurMean(9:12));
        Seva_p.Jparams.Rself=diag(xPurMean(13:14));
        Seva_p.Jparams.Ropp=zeros(2,2);
        % Propagate to next time step
        [up,ue,flag]=f_dyn(Spur_p,Seva_p,gameState_p,zeros(4,1));
        if flag==0
            upp=randsample(1:length(Spur_p.uMat),1,true,up);
            uPurTrue=Spur_p.uMat{upp}(:,1);
            uEvaEst=zeros(gameState_p.nu,gameState_p.kMax);
            for ik=1:length(Seva_p.uMat)
                uEvaEst=uEvaEst+ue(ik)*Seva_p.uMat{ik}(:,1);
            end
            QinflatePur=Qnoisepur+blkdiag(zer2,1*eye2);
        else
            uPurTrue=up;
            uEvaEst=ue;
        end
    end
    xPur_bar(1:4)=f_dynPur(xPurMean(1:4),uPurTrue,tstep,zeros(2,1));
    xPur_bar(5:8)=f_dynEva(xPurMean(5:8),uEvaEst ,tstep,zeros(2,1));
    
    
    %Omniscient evader
    gameState_e.xPur=xEva(1:4);
    gameState_e.xEva=xEva(5:8);
    gameState_e.dt=tstep;
    gameState_e.kMax=1;
    gameState_e.nu=2;
    Spur_e.uMat={0}; Seva_e.uMat={0};
    for ik=1:length(utemp)
        Spur_e.uMat{ik}=utemp(:,ik);
    end
    Spur_e.Jname='J_pur';
    Spur_e.fname='f_dynPur';
    Spur_e.Jparams.Q=Qpur;
    Spur_e.Jparams.Rself=Rpur;
    Spur_e.Jparams.Ropp=zeros(2,2);
    Seva_e.uMat = Spur_e.uMat;
    Seva_e.Jname='J_eva';
    Seva_e.fname='f_dynEva';
    Seva_e.Jparams.Q=Qeva;
    Seva_e.Jparams.Rself=Reva;
    Seva_e.Jparams.Ropp=zeros(2,2);    
    gameState_e = gameState_p;
    gameState_e.xPur=xEva(1:4);
    gameState_e.xEva=xEva(5:8);
    if evaderIsOblivious
        uPurEst=zeros(2,1);
        uEvaTrue=upmax/3*traveldirEva;
    else
        [up,ue,flag]=f_dyn(Spur_e,Seva_e,gameState_e,zeros(4,1));
        if flag==0
            uee=randsample(1:length(Seva_e.uMat),1,true,ue);
            uEvaTrue=Seva_e.uMat{uee}(:,1);
            uPurEst=zeros(gameState_p.nu,gameState_p.kMax);
            for ik=1:length(Spur_e.uMat)
                uPurEst=uPurEst+up(ik)*Spur_e.uMat{ik}(:,1);
            end
            QinflateEva=Qnoiseeva+blkdiag(1*eye2, zer2);
        else
            uEvaTrue=ue;
            uPurEst=up;
        end
    end
    
    xTrue(1:4)=f_dynPur(xTrue(1:4),uPurTrue(:,1),tstep,zeros(2,1));
    xTrue(5:8)=f_dynEva(xTrue(5:8),uEvaTrue(:,1),tstep,zeros(2,1));

    % Measurement
    zPur=Hpur*xTrue+chol(Rnoisepur)'*randn(4,1);
    zEva=Heva*xTrue+chol(Rnoiseeva)'*randn(4,1);
    
    [xPur,Ppur]=kfstep(xPur,zPur,Aeva,Bnoiseeva,[uPurTrue;uEvaEst],Gnoiseeva,QinflatePur,Ppur,Heva,Rnoiseeva);
    [xEva,Peva]=kfstep(xEva,zEva,Aeva,Bnoiseeva,[uPurEst;uEvaTrue],Gnoiseeva,QinflateEva,Peva,Heva,Rnoisepur);
    xTrueS{n+1}=xTrue;
    
    %cost calculation, need to debug error index length in J_pur
    e=xTrue(1:4)-xTrue(5:8);
    Jloc = e'*Spur_p.Jparams.Q*e + uPurTrue'*Spur_p.Jparams.Rself*uPurTrue + ...
        uEvaTrue'*Spur_p.Jparams.Ropp*uEvaTrue;
    Jp0=Jp0+Jloc;
    
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
    plot(xE(1,:),xE(2,:),'-or');
    hold on
    plot(xP(1,:),xP(2,:),'-xb');
    figset
    load temp.mat
    plot(xPtilde(1,:),xPtilde(2,:),'-xk');
    xlabel('East displacement (m)');
    ylabel('North displacement (m)');
    legend('Evader','Pursuer, GT','Pursuer, VM');
    axis([0 90 0 10])
    figset
end

