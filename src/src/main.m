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
%npart=5000;
npart=500;

tstep=1;
tmax=20;

%utemp=permn(-2:.5:2,2)';
utemp=permn(-2:0.5:2,2)';

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
xTrue=[[0 0 2.2 0.8]'; [4 6 0 0]'];
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
    xEva=xEva;
%    xPurMean=xTrue+1e-2*randn(8,1);
%    xEva=xTrue+1e-2*randn(8,1);
    
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
    Spur_p.fname='f_dynPur';
    Spur_p.Jparams.Q=Qpur;
    Spur_p.Jparams.Rself=Rpur;
    Spur_p.Jparams.Ropp=zeros(2,2);
    Seva_p.uMat = Spur_p.uMat;
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
    xPur_bar(1:4)=f_dynPur(xPurMean(1:4),uPurTrue,tstep,zeros(2,1));
    xPur_bar(5:8)=f_dynEva(xPurMean(5:8),uEvaEst ,tstep,zeros(2,1));
    
    uPstore={};uEstore={};
    
    % Propagate particles
    for ik=1:npart
        %ik
        gameState_p.xPur=xPur_part(1:4,ik);
        gameState_p.xEva=xPur_part(5:8,ik);
        Seva_p.Jparams.Q=diag(xPur_part(9:12,ik));
        Seva_p.Jparams.Rself=diag(xPur_part(13:14,ik));
        [up,ue,flag]=f_dyn(Spur_p,Seva_p,gameState_p,zeros(4,1));
        if flag==0
            upp=randsample(1:length(Spur_p.uMat),1,true,up);
            uPurThis=Spur_p.uMat{upp}(:,1);
            uEvaThis=zeros(gameState_p.nu,gameState_p.kMax);
            for iL=1:length(Seva_p.uMat)
                uEvaThis=uEvaThis+ue(iL)*Seva_p.uMat{iL}(:,1);
            end
            QinflatePur=Qnoisepur+blkdiag(zer2,1*eye2);
        else
            uPurThis=up;
            uEvaThis=ue;
        end
        uPstore{ik}=uPurThis;
        uEstore{ik}=uEvaThis;
%        dxEx=xPur(5:8)-f_dynPur(xPur(5:8),uEvaThis,tstep,zeros(2,1))
        xPur_part(1:4,ik)=f_dynPur(xPur_part(1:4,ik),uPurThis,tstep,cholQ2p*randn(2,1));
        xPur_part(5:8,ik)=f_dynEva(xPur_part(5:8,ik),uEvaThis,tstep,cholQ2e*randn(2,1));
        xPur_part(9:12,ik)=diag(Seva_p.Jparams.Q).*10.^(cholQexcite*(rand(4,1)*2-1));
        xPur_part(13:14,ik)=diag(Seva_p.Jparams.Rself).*10.^(cholRexcite*(rand(2,1)*2-1));
        for iL=1:6
            rn=rand;
            if(rn<=cholExcitePlusOne && xPur_part(8+iL,ik)<1e-8) %breaking out of zero
                xPur_part(8+iL,ik)=xPur_part(8+iL,ik)+1;
            elseif (rn<=cholExciteDropOrder && xPur_part(8+iL,ik)>1e-8)
                xPur_part(8+iL,ik)=xPur_part(8+iL,ik)*10^-orderExcite;
            elseif (rn<=2*cholExciteDropOrder)
                xPur_part(8+iL,ik)=xPur_part(8+iL,ik)*10^orderExcite;
            end
            if xPur_part(8+iL,ik)>10^Qmax
                xPur_part(8+iL,ik)=10^Qmax;
            end
        end
    end
    xkbar=zeros(14,1);
    for ik=1:npart
        xkbar=xkbar+w_set_pf(ik,n)*xPur_part(:,ik);
    end
    Pkbar=zeros(14,14);
    for ik=1:npart
        Pkbar=Pkbar+w_set_pf(ik,n)*(xPur_part(:,ik)-xkbar)*(xPur_part(:,ik)-xkbar)';
    end
    wloc=w_set_pf(:,n);
    wloc=wloc/sum(wloc);
    
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
    
    xTrue(1:4)=f_dynPur(xTrue(1:4),uPurTrue(:,1),tstep,zeros(2,1));
    xTrue(5:8)=f_dynEva(xTrue(5:8),uEvaTrue(:,1),tstep,zeros(2,1));
%     uPur=uPurTrue
%     uEva=uEvaTrue
%     xTrue

    % Measurement
    zPur=Hpur*xTrue+chol(Rnoisepur)'*randn(4,1);
    zEva=Heva*xTrue+chol(Rnoiseeva)'*randn(4,1);
    
    [xPur,Ppur]=kfstep(xPur,zPur,Aeva,Bnoiseeva,[uPurTrue;uEvaEst],Gnoiseeva,QinflatePur,Ppur,Heva,Rnoiseeva);
    [xEva,Peva]=kfstep(xEva,zEva,Aeva,Bnoiseeva,[uPurEst;uEvaTrue],Gnoiseeva,QinflateEva,Peva,Heva,Rnoisepur);
    
    for ik=1:npart
        Sk=H14*Pkbar*H14'+Rnoisepur;
        zhat=H14*xPur_part(:,ik);
        nu=zPur-zhat;
        wloc(ik)=w_set_pf(ik,n)*mvnpdf(nu,zeros(4,1),(Sk+Sk')/2)+1e-10;
        %wloc(ik)
    end
    wloc = wloc/sum(wloc);
    meann=zeros(14,1);
    for ik=1:npart
        meann=meann+wloc(ik)*xPur_part(:,ik);
    end
    %dx=xPur-meann(1:8)
    dJ=meann(9:14)-qrTrue;
    dJS(:,n+1)=dJ;
    
    % Recenter particles, not technically correct but I'm going to see if
    %   the performance improves
%     for ik=1:npart
%         xPur_part(1:8,ik)=xPur;
%     end
    
    %Resample if necessary
%     Neff=1/sum(wloc.^2);
%     if Neff<=npart/2
    ind=sysresample(wloc);
    wloc = wloc(ind)/sum(wloc(ind));
    xPur_part(:,:)=xPur_part(:,ind);
    wStore{n+1}=wloc;
    xPartStore{n+1}=xPur_part;
    xPurS{n+1}=xPur;
    xEvaS{n+1}=xEva;
    xTrueS{n+1}=xTrue;
%     end
    
    w_set_pf = [w_set_pf wloc];

    
%    pErr=xPur-xTrue
%    eErr=xEva-xTrue
    
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
    figure(2);clf;
    figset
    subplot(3,1,1);
    plot(1:n+1,dJS(1,:),'-.r');
    hold on
    plot(1:n+1,dJS(2,:),'-ob');
    legend('\DeltaQ_{xx}','\DeltaQ_{yy}')
    xlabel('Time (s)');
    ylabel('Cost parameter (m^{-2})');
    figset
    
    subplot(3,1,2);
    plot(1:n+1,dJS(3,:),'-.r');
    hold on
    plot(1:n+1,dJS(4,:),'-ob');
    legend('\DeltaQ_{vx}','\DeltaQ_{vy}');
    xlabel('Time (s)');
    ylabel('Cost parameter (m^{-2}s^2)');
    figset
    
    subplot(3,1,3);
    plot(1:n+1,dJS(5,:),'-.r');
    hold on
    plot(1:n+1,dJS(6,:),'-ob');
    legend('\DeltaR_{x}','\DeltaR_{y}');
    xlabel('Time (s)');
    ylabel('Cost parameter (m^{-2}s^4)');
    figset
    
    xP=zeros(2,n+1);xE=zeros(2,n+1);
    for ijk=1:n+1
        xP(:,ijk)=xTrueS{ijk}(1:2); xE(:,ijk)=xTrueS{ijk}(5:6);
    end
    figure(3);clf;
    plot(xP(1,:),xP(2,:),'-xr');
    hold on
    plot(xE(1,:),xE(2,:),'-ob');
    xlabel('East displacement (m)');
    ylabel('North displacement (m)');
    legend('Pursuer','Evader');
    figset
    
end
