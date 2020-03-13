clear;clc;

% Game state convention
%        +x
%         |
%         |
%   +y - -
%

%GT: J=6.5432e+03
%VM: J=9.7911e+03
%NN: J=1.5014e+04

%GT: J=8.4332e+03
%VM: J=9.2094e+03
%NN: J=1.6373e+04

%GT: J=8.7651e+03
%VM: J=1.5638e+04
%NN: J=2.5165e+04

%note: change upmax to 1 for second go

load pursuerTestNet.mat
network1=net;

%rngseedno=10;
rngseedno=40;
rng(rngseedno)

plotFlag=0;
plotEndFlag=1;
%npart=5000;
npart=500;

% REWRITE CONTROL TYPE CODE
%Evader control type info
evaderIsOblivious=0; %if true, evader uses preset control
evaderUsesGT=1;
evaderUsesKumar=0;

%Pursuer control type info
pursuerUsesVelmatch=0;
pursuerUsesGT=1;
useNNforGT=0;
pursuerUsesKumar=0;

%tuning params
safeDecelParamVM=0.45; %percentage of throttle to dedicate to excess thrust

%Will velmatching use a control estimate?
%uEvaBestPerformanceEstimate=[0.9;0]; %lower can produce oscillations
uEvaBestPerformanceEstimate=[0;0]; 

tstep=1;
tmax=5;

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

Qpur = diag([50 20 10 20]); Rpur = diag([90 80]);
Qeva = diag([50 30 20 20]); Reva = diag([10 10]);
%cdP=.3*rand;
%cdE=.3*rand;
cdP=0.2;
cdE=0.2;

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

if pursuerUsesKumar || evaderUsesKumar
    Gcontinuous=[0 0
        0 0
        1 0
        0 1];
    [Fcontinuous,Aapprx]=calculateFforKumar(Abk2,tstep,[1 0 1 0; 0 1 0 1; .01 0 -.21 0; 0 .01 0 -.21]); %NOTE:these are subtracted
end

Ppur=Peva;
%This can be shunted off to a separate function
n=0; %n is now the iteration index (yes, misuse of var names)
xTrue=[[0 0 0.3 0.1]'; [4 2 0 0]'];
xPur=xTrue;
xEva=xTrue;
axisveck=[-20 20 -5 35];

gameStateValsEva.nX=4;
gameStateValsEva.R21=zeros(2,2);
gameStateValsEva.R12=zeros(2,2);
if pursuerUsesKumar || evaderUsesKumar
    gameStateValsEva.F=Fcontinuous;
    gameStateValsEva.G1=Gcontinuous;
    gameStateValsEva.G2=Gcontinuous;
end
gameStateValsEva.W=Qnoiseeva;
gameStateValsEva.V1=Peva(1:4,1:4);
gameStateValsEva.V2=Ppur(1:4,1:4);
gameStateValsEva.R22=Reva;
gameStateValsEva.R11=Rpur;
gameStateValsEva.H1=eye(4);
gameStateValsEva.H2=eye(4);
gameStateValsEva.umax=umax;
Pvec0Eva=repmat(reshape(Qnoiseeva+Qnoisepur,[16,1]),[3 2]);
Qvec0Eva=[[reshape(Qeva, [16 1]); reshape(Qpur, [16,1]); zeros(32,1)] [reshape(Qeva, [16 1]); reshape(Qpur, [16,1]); zeros(32,1)]];

gameStateValsPur.nX=4;
gameStateValsPur.R21=zeros(2,2);
gameStateValsPur.R12=zeros(2,2);
if pursuerUsesKumar || evaderUsesKumar
    gameStateValsPur.F=Fcontinuous;
    gameStateValsPur.G1=Gcontinuous;
    gameStateValsPur.G2=Gcontinuous;
end
gameStateValsPur.W=Qnoiseeva;
gameStateValsPur.V1=Ppur(1:4,1:4);
gameStateValsPur.V2=Peva(1:4,1:4);
gameStateValsPur.R22=Rpur;
gameStateValsPur.R11=Reva;
gameStateValsPur.H1=eye(4);
gameStateValsPur.H2=eye(4);
gameStateValsPur.umax=umax;
Pvec0Pur=repmat(reshape(Qnoiseeva+Qnoisepur,[16,1]),[3 2]);
Qvec0Pur= -[[reshape(Qpur, [16 1]); reshape(Qeva, [16,1]); zeros(32,1)] [reshape(Qpur, [16 1]); reshape(Qeva, [16,1]); zeros(32,1)]];

if evaderUsesKumar || pursuerUsesKumar
tvec0pPur=[0 tstep]; tvec0pEva=[0 tstep];
tvec0qPur=[tstep 0]; tvec0qEva=[tstep 0];
for ik=1:3 %fwd/bwd pass three times
    propagateP_Pur=@(t,x) odeKumarP(t,x,Qvec0Pur,tvec0qPur,gameStateValsPur);
    [tvec0pPur,Pvec0Pur]=ode45(propagateP_Pur,[0 tstep],Pvec0Pur(:,1));
    Pvec0Pur=Pvec0Pur';
    
    propagateQ_Pur=@(t,x) odeKumarQ(t,x,Pvec0Pur,tvec0pPur,gameStateValsPur);
    [tvec0qPur,Qvec0Pur]=ode45(propagateQ_Pur,[tstep 0],Qvec0Pur(:,1));
    Qvec0Pur=Qvec0Pur';
    %NOTE: CHECK THAT OUTPUTS ARE IN THE RIGHT TIME SERIES ORDER
end

%Created separate loops for P and E for faster debugging
for ik=1:3 %fwd/bwd pass three times
    propagateP_Eva=@(t,x) odeKumarP(t,x,Qvec0Eva,tvec0qEva,gameStateValsEva);
    [tvec0pEva,Pvec0Eva]=ode45(propagateP_Eva,[0 tstep],Pvec0Eva(:,1));
    Pvec0Eva=Pvec0Eva';
    
    propagateQ_Eva=@(t,x) odeKumarQ(t,x,Pvec0Eva,tvec0pEva,gameStateValsEva);
    [tvec0qEva,Qvec0Eva]=ode45(propagateQ_Eva,[tstep 0],Qvec0Eva(:,1));
    Qvec0Eva=Qvec0Eva';
    %NOTE: CHECK THAT OUTPUTS ARE IN THE RIGHT TIME SERIES ORDER
end
end

if plotFlag==1
figure(1);clf
f1=scatter(xTrue(1),xTrue(2),'b');
hold on
f2=scatter(xTrue(5),xTrue(6),'r');
axis(axisveck)
end

%Particle filter initialization
xPur_part=zeros(14,npart);
for ij=1:npart
    xPur_part(1:8,ij)=xTrue+chol(Ppur)'*randn(8,1);
end
Qmax = 2.5; %10^order
Qmin = 0.8;
xPur_part(9:14,:)=randpe(6,npart,0.35,Qmin,Qmax);
xbarprev=mean(xPur_part,2);
w_set_pf=1/npart*ones(npart,1);
wloc=w_set_pf;

% Scaling on excitation for Jparams
cholQexcite=0.05;
cholRexcite=0.05;
% P to do thing
cholExcitePlusOne=0.05;
cholExciteDropOrder=0.00;
orderExcite=0.5;

wStore{1}=wloc;
xPartStore{1}=xPur_part;
xPurS{1}=xPur;
xEvaS{1}=xEva;
xTrueS{1}=xTrue;
dJS=[];
Jp0=0;
xBlck=[];
uBlck=[];

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
    
    xPurMean=[xPur;diag(Qeva);diag(Reva)];
    
    %Preload GT params used in one or more GT solvers
    gameState_p.xPur=xPurMean(1:4);
    gameState_p.xEva=xPurMean(5:8);
    gameState_p.dt=tstep;
    gameState_p.kMax=1;
    gameState_p.nu=2;
    uvelmatch=vmRGVO_tune(xPur(1:4),xPur(5:8),upmax,2,tstep,uEvaBestPerformanceEstimate,safeDecelParamVM);
    for ik=1:length(utemp)
        Spur_p.uMat{ik}=utemp(:,ik);
    end
    Spur_p.Jname='J_pur';
    Spur_p.fname='f_dynCD2';
    Spur_p.Jparams.Q=Qpur;
    Spur_p.Jparams.Rself=Rpur;
    Spur_p.Jparams.Ropp=zeros(2,2);
    Seva_p.uMat = Spur_p.uMat;
    Spur_p.uMat{length(utemp)+1}=uvelmatch;
    Seva_p.Jname='J_eva';
    Seva_p.fname='f_dynCD2';
    Seva_p.Jparams.Q=diag(xPurMean(9:12));
    Seva_p.Jparams.Rself=diag(xPurMean(13:14));
    Seva_p.Jparams.Ropp=zeros(2,2);
    Spur_p.params.cd=cdP;
    Seva_p.params.cd=cdE;
    if pursuerUsesVelmatch
        uPurTrue=uvelmatch;
        uEvaEst=zeros(2,1);
    elseif pursuerUsesGT || useNNforGT %lazy
        if useNNforGT
            qqrr=[diag(Qpur); diag(Qeva); diag(Rpur); diag(Reva);cdP;cdE];
            ministate=xPurMean;
            uStack=predict(network1,[xPur;qqrr]);
            uStack=double(uStack);
            uPurTrue=uStack(1:2)';
%             uEvaEst=uStack(3:4)';
            uEvaEst=[0;0];
        else
            % Guessing pursuer
            gameState_p.xPur=xPurMean(1:4);
            gameState_p.xEva=xPurMean(5:8);
            gameState_p.dt=tstep;
            gameState_p.kMax=1;
            gameState_p.nu=2;
            for ik=1:length(utemp)
                Spur_p.uMat{ik}=utemp(:,ik);
            end
            Spur_p.Jname='J_pur';
            Spur_p.fname='f_dynCD2';
            Spur_p.Jparams.Q=Qpur;
            Spur_p.Jparams.Rself=Rpur;
            Spur_p.Jparams.Ropp=zeros(2,2);
            Spur_p.params.cd=cdP;
            Seva_p.uMat = Spur_p.uMat;
            Spur_p.uMat{length(utemp)+1}=uvelmatch;
            Seva_p.Jname='J_eva';
            Seva_p.fname='f_dynCD2';
            Seva_p.Jparams.Q=diag(xPurMean(9:12));
            Seva_p.Jparams.Rself=diag(xPurMean(13:14));
            Seva_p.Jparams.Ropp=zeros(2,2);
            Seva_p.params.cd=cdE;
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
    end
    if pursuerUsesGT || pursuerUsesVelmatch
        xPur_bar(1:4)=f_dynCD2(xPurMean(1:4),uPurTrue,tstep,zeros(2,1),Spur_p.params);
        xPur_bar(5:8)=f_dynCD2(xPurMean(5:8),uEvaEst ,tstep,zeros(2,1),Seva_p.params);
    end
    
    if evaderUsesGT
        %Omniscient evader
        gameState_e.xPur=xEva(1:4);
        gameState_e.xEva=xEva(5:8);
        gameState_e.dt=tstep;
        gameState_e.kMax=1;
        gameState_e.nu=2;
        for ik=1:length(utemp)
            Spur_e.uMat{ik}=utemp(:,ik);
        end
        Spur_e.Jname='J_pur';
        Spur_e.fname='f_dynCD2';
        Spur_e.Jparams.Q=Qpur;
        Spur_e.Jparams.Rself=Rpur;
        Spur_e.Jparams.Ropp=zeros(2,2);
        Spur_e.params.cd=cdP;
        Seva_e.uMat = Spur_e.uMat;
        Seva_e.Jname='J_eva';
        Seva_e.fname='f_dynCD2';
        Seva_e.Jparams.Q=Qeva;
        Seva_e.Jparams.Rself=Reva;
        Seva_e.Jparams.Ropp=zeros(2,2);
        Seva_e.params.cd=cdE;
        gameState_e = gameState_p;
        gameState_e.xPur=xEva(1:4);
        gameState_e.xEva=xEva(5:8);
        if evaderIsOblivious
            uPurEst=zeros(2,1);
            uEvaTrue=upmax/3*traveldirEva; %uemax=upmax
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
    end
    uEvaTrue;
    
    if pursuerUsesKumar
        ff=@(t,x) dynamicsKumar(t,x,Qvec0Pur,tvec0qPur,Fcontinuous,Gcontinuous,gameStateValsPur);
        [ttemp,xtemp]=ode45(ff,[0 tstep],[xTrue(1:4);xTrue(5:8)]);
        xTrue(1:4) = xtemp(end,1:4)';
    else
        xTrue(1:4)=f_dynCD2(xTrue(1:4),uPurTrue(:,1),tstep,zeros(2,1),Spur_p.params);
    end
    
    if evaderUsesGT
        xTrue(5:8)=f_dynCD2(xTrue(5:8),uEvaTrue(:,1),tstep,zeros(2,1),Seva_e.params);
    elseif evaderUsesKumar
        ff=@(t,x) dynamicsKumar(t,x,Qvec0Eva,tvec0qEva,Fcontinuous,Gcontinuous,gameStateValsEva);
        [ttemp,xtemp]=ode45(ff,[0 tstep],[xTrue(5:8);xTrue(1:4)]);
        xTrue(5:8) = xtemp(end,1:4)';        
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
    xBlck=[xBlck xTrue];
    uBlck=[uBlck [uPurTrue;uEvaTrue]];
    
    %cost calculation, need to debug error index length in J_pur
    e=xTrue(1:4)-xTrue(5:8);
    Jloc = e'*Spur_p.Jparams.Q*e + uPurTrue'*Spur_p.Jparams.Rself*uPurTrue + ...
        uEvaTrue'*Spur_p.Jparams.Ropp*uEvaTrue;
    Jp0=Jp0+Jloc
    
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
