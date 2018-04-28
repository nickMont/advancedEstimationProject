clear;clc;
% Main function for 2D game AI for pop-the-balloon

%
% Game state convention
%        +x
%         |
%         |
%   +y - -
%

rng(10)

plotFlag=0;

tstep=1;
tmax=20;

Game.uType="velstep";
%Options: velstep (step of constant size, from velocity)
%         accel (double integrator)
Game.dt=tstep;

% Red and blue teams, Sred and Sblue
% S contains state information

Qpur = diag([100 100 0 0]); Rpur = diag([0.1 1]);
Qeva = diag([10 10 0 0]); Reva = diag([1 1]);

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
Abk1=[eye(n) dt*eye(n); zeros(n,n) 0.35*eye(n)]; %Apur
Abk2=[eye(n) dt*eye(n); zeros(n,n) 0.3*eye(n)]; %Aeva
Aeva=blkdiag(Abk1,Abk2);
Gnoiseeva=blkdiag([eye(n)*dt^2/2;eye(n)*dt],[eye(n)*dt^2/2;eye(n)*dt]);
Qnoiseeva=0.002*eye(4);
Qnoisepur=Qnoiseeva;
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
npart=50;
xPur_part=zeros(14,npart);
for ij=1:npart
    xPur_part(1:8,ij)=xTrue+chol(Ppur)'*randn(8,1);
end
xPur_part(9:14,1:npart)=randpe(6,npart,0.35,0.8,2.2);
w_set_pf=1/npart*ones(npart,1);
wloc=zeros(npart,1);

cholQexcite=0.15;
cholRexcite=0.15;
cholExcitePlusOne=0.05;
cholExciteDropOrder=0.05;

for ij=1:tstep:tmax
    n=n+1;
    
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
    utemp=permn(-2:.5:1,2)';
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
    
    % Propagate particles
    for ik=1:npart
        Seva_p.Jparams.Q=diag(xPur_part(9:12));
        Seva_p.Jparams.Rself=diag(xPur_part(13:14));
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
        xPur_part(1:4,ik)=f_dynPur(xPurMean(1:4),uPurTrue,tstep,cholQ2p*randn(2,1));
        xPur_part(5:8,ik)=f_dynEva(xPurMean(5:8),uEvaEst ,tstep,cholQ2e*randn(2,1));
        xPur_part(9:12,ik)=diag(Seva_p.Jparams.Q).*10.^(cholQexcite*(rand(4,1)*2-1));
        xPur_part(13:14,ik)=diag(Seva_p.Jparams.Rself).*10.^(cholRexcite*(rand(2,1)*2-1));
        for iL=1:6
            if(cholExcitePlusOne<=rand) %breaking out of zero
                xPur_part(8+iL,ik)=xPur_part(8+iL,ik)+1;
            elseif (cholExciteDropOrder<=rand)
                xPur_part(8+iL,ik)=xPur_part(8+iL,ik)*0.1;
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
    
    %Omniscient evader
    gameState_e.xPur=xEva(1:4);
    gameState_e.xEva=xEva(5:8);
    gameState_e.dt=tstep;
    gameState_e.kMax=1;
    gameState_e.nu=2;
    utemp=permn(-2:.5:1,2)';
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
        for ik=1:length(Seva_p.uMat)
            uPurEst=uPurEst+up(ik)*Seva_p.uMat{ik}(:,1);
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
    end
    wloc = wloc/sum(wloc);
    meann=zeros(14,1);
    for ik=1:npart
        meann=meann+wloc(ik)*xPur_part(:,ik);
    end
    dx=xPur-meann(1:8)
    dJ=qrTrue-meann(9:14)

        
    %Resample if necessary
    ind=sysresample(wloc);
    wloc = wloc(ind)/sum(wloc(ind));
    xPur_part(:,:)=xPur_part(:,ind);
    
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
    
end



