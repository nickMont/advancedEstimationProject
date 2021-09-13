clear;clc;loadenv;
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
%npart=5000;
npart=500;

%Evader control type info
evaderIsOblivious=false; %if true, evader uses preset control
evaderUsesGT=true;
evaderUsesKumar=false;

%Pursuer control type info
pursuerUsesVelmatch=false;
pursuerUsesGT=true;
pursuerUsesKumar=true;
useNNforGT=false;

%Will velmatching use a control estimate?
%uEvaBestPerformanceEstimate=[0.9;0]; %lower can produce oscillations
uEvaBestPerformanceEstimate=[0;0]; 

tstep=1;
tmax=20;

%utemp=permn(-2:.5:2,2)';
%utemp=permn(-2:0.5:2,2)';
utemp=permn(-1:0.5:1,2)';
upmax=2;
umax=upmax;
utype.radius=upmax;
utemp=mapUtempToUvec(utemp,"circle",utype);
max_steps_to_predict=1;
utemp_perm = permuteOverTime(utemp,max_steps_to_predict);

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
if usesKumar
Gcontinuous1=[0 0
              1 0
              0 0
              0 1];
Gcontinuous2=[0 0
              1 0
              0 0
              0 1];          
Fcontinuous=[0 1 0 0
             0 0 0 0
             0 0 0 1
             0 0 0 0];
end

%calculate state transition matrix here
A=expm(Fcontinuous*dt);
%Note: B,u continuous over time step, so zero-state response is just
%  the (integral of the state transition matrix)*B*u
B1 = (expm(Fcontinuous*dt)-eye(4))*inv(A)*Gcontinuous1;
B2 = (expm(Fcontinuous*dt)-eye(4))*inv(A)*Gcontinuous2;

Ppur=Peva;
%This can be shunted off to a separate function
n=0; %n is now the iteration index (yes, misuse of var names)
xTrue=[[0 0 1.0 0.8]'; [10 4 0 0]'];
xPur=xTrue;
xEva=xTrue;
axisveck=[-20 20 -5 35];

gameStateValsEva.nX=4;
gameStateValsEva.R21=zeros(2,2);
gameStateValsEva.R12=zeros(2,2);
if usesKumar
    gameStateValsEva.F=Fcontinuous;
    gameStateValsEva.G1=Gcontinuous1;
    gameStateValsEva.G2=Gcontinuous2;
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
if usesKumar
    gameStateValsPur.F=Fcontinuous;
    gameStateValsPur.G1=Gcontinuous1;
    gameStateValsPur.G2=Gcontinuous2;
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

tvec0pPur=[0 tstep]; tvec0pEva=[0 tstep];
tvec0qPur=[tstep 0]; tvec0qEva=[tstep 0];
if usesKumar
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
%     for ik=1:3 %fwd/bwd pass three times
%         propagateP_Pur=@(t,x) odeKumarP(t,x,Qvec0Pur,tvec0qPur,gameStateValsPur);
%         [tvec0pPur,Pvec0Pur]=ode45(propagateP_Pur,[0 tstep],Pvec0Pur(:,1));
%         Pvec0Pur=Pvec0Pur';
%     
%         propagateQ_Pur=@(t,x) odeKumarQ(t,x,Pvec0Pur,tvec0pPur,gameStateValsPur);
%         [tvec0qPur,Qvec0Pur]=ode45(propagateQ_Pur,[tstep 0],Qvec0Pur(:,1));
%         Qvec0Pur=Qvec0Pur';
%         %NOTE: CHECK THAT OUTPUTS ARE IN THE RIGHT TIME SERIES ORDER
%     end
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
    gameState_p.kMax=max_steps_to_predict;
    gameState_p.nu=2;
    uvelmatch=vmRGVO_max(xPur(1:4),xPur(5:8),upmax,2,tstep,uEvaBestPerformanceEstimate);
    Spur_p.uMat={0}; Seva_p.uMat={0};
    for ik=1:max(size(utemp_perm))
        Spur_p.uMat{ik}=squeeze(utemp_perm(:,:,ik));
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
    elseif pursuerUsesGT
        if useNNforGT
            qqrr=0.1*[diag(Qpur); diag(Qeva); diag(Rpur); diag(Reva)];
            ministate=xPurMean;
            uStack=predict(network1,[ministate;qqrr]);
            uPurTrue=uStack(1:2);
            uEvaEst=uStack(3:4);
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
    end
    if pursuerUsesGT || pursuerUsesVelmatch
        xPur_bar(1:4)=f_dynPur(xPurMean(1:4),uPurTrue,tstep,zeros(2,1));
        xPur_bar(5:8)=f_dynEva(xPurMean(5:8),uEvaEst ,tstep,zeros(2,1));
    end
    
    if evaderUsesGT
        %Omniscient evader
        gameState_e.xPur=xEva(1:4);
        gameState_e.xEva=xEva(5:8);
        gameState_e.dt=tstep;
        gameState_e.kMax=max_steps_to_predict;
        gameState_e.nu=2;
        Spur_e.uMat={0}; Seva_e.uMat={0};
        for ik=1:max(size(utemp_perm))
            Spur_e.uMat{ik}=squeeze(utemp_perm(:,:,ik));
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
    end
    
    if pursuerUsesGT || pursuerUsesVelmatch
        xTrue(1:4)=f_dynPur(xTrue(1:4),uPurTrue(:,1),tstep,zeros(2,1));
    elseif pursuerUsesKumar
        ff=@(t,x) dynamicsKumar(t,x,Qvec0Pur,tvec0qPur,Fcontinuous,Gcontinuous1,gameStateValsPur);
        [ttemp,xtemp]=ode45(ff,[0 tstep],[xTrue(1:4);xTrue(5:8)]); %#ok
        xTrue(1:4) = xtemp(end,1:4)';
    end
    
    if evaderUsesGT
        xTrue(5:8)=f_dynEva(xTrue(5:8),uEvaTrue(:,1),tstep,zeros(2,1));
    elseif evaderUsesKumar
        ff=@(t,x) dynamicsKumar(t,x,Qvec0Eva,tvec0qEva,Fcontinuous,Gcontinuous2,gameStateValsEva);
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
    
%     %cost calculation, need to debug error index length in J_pur
    e=xTrue(1:4)-xTrue(5:8);
    Jloc = e'*Spur_p.Jparams.Q*e + uPurTrue(:,1)'*Spur_p.Jparams.Rself*uPurTrue(:,1) + ...
        uEvaTrue(:,1)'*Spur_p.Jparams.Ropp*uEvaTrue(:,1);
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
