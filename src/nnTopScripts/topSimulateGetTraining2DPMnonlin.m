clear;clc;

%
% Game state convention
%        +x
%         |
%         |
%   +y - -
%

%NOTE: CURRENTLY SET FOR U BASED ON VELOCITY MATCHING SCALE

%load nnvarDynTrained0417.mat
%network1=net;
testNN=false;

iter=100;

ueS_S=cell(iter,1);
upS_S=cell(iter,1);
xtrueS_S=cell(iter,1);
qrS_S=cell(iter,1);
scaleS_S=cell(iter,1);

%rngseedno=457;
%rng(rngseedno)

for iteriter=1:iter

completion_status = iteriter/iter*100
    
plotFlag=0;

tstep=1;
tmax=20;


%utemp=permn(-2:0.2:2,2)';
uvec=-1:0.05:1;
utemp=permn(uvec,2)';
upmax=2;
umax=upmax;
utype.radius=upmax;
utemp=mapUtempToUvec(utemp,"circle",utype);

uEvaBestPerformanceEstimate=[0;0];
safeDecelParamVM=0.2;

Game.uType="velstep";
%Options: velstep (step of constant size, from velocity)
%         accel (double integrator)
Game.dt=tstep;

%Qpur = diag([100 100 0 0]); Rpur = diag([0.1 1]);
%Qeva = diag([100 50 0 0]); Reva = diag([5 10]);

Qpur = diag(100*rand(1,4)); Rpur = diag(100*rand(1,2));
Qeva = diag(100*rand(1,4)); Reva = diag(100*rand(1,2));

qrTrue=[diag(Qeva);diag(Reva)];
cdP=.3*rand;
cdE=.3*rand;
qrTrueAll=[diag(Qpur); diag(Rpur); diag(Qeva); diag(Reva);cdP;cdE];

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
%xTrue=[[0 0 2.2 0.8]'; [4 6 0 0]'];
xTrue = diag([20 20 2 2 20 20 2 2])*(2*rand(8,1)-1);
%xTrue = diag([4 4 .5 .5 4 4 .5 .5])*rand(8,1);
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

xTrueS={};
xTrueS{1}=xTrue;
upS=cell(floor(tmax/tstep),1);ueS=cell(floor(tmax/tstep),1);
scaleS=cell(floor(tmax/tstep),1);

for ij=1:tstep:tmax
    n=n+1
    tic
    
    %reset noise inflations
    QinflatePur=Qnoisepur;
    QinflateEva=Qnoiseeva;
    scaleS{n}=zeros(2,1);
    
    xPurMean=[xPur;diag(Qeva);diag(Reva)];
    xEva=xEva;
%    xPurMean=xTrue+1e-2*randn(8,1);
%    xEva=xTrue+1e-2*randn(8,1);
    
    % Guessing pursuer
    gameState_p.xPur=xTrue(1:4);
    gameState_p.xEva=xTrue(5:8);
    gameState_p.dt=tstep;
    gameState_p.kMax=1;
    gameState_p.nu=2;
    Spur_p.uMat={0}; Seva_p.uMat={0};
    uhat=unit_vector(vmRGVO_tune(xTrue(1:4),xTrue(5:8),upmax,2,tstep,uEvaBestPerformanceEstimate,safeDecelParamVM));
    for ik=1:length(uvec)
        Spur_p.uMat{ik}=upmax*uvec(ik)*uhat;
    end
%     for ik=1:length(utemp)
%         Spur_p.uMat{ik}=utemp(:,ik);
%     end
    Spur_p.Jname='J_pur';
    Spur_p.fname='f_dynCD2';
    Spur_p.Jparams.Q=Qpur;
    Spur_p.Jparams.Rself=Rpur;
    Spur_p.Jparams.Ropp=zeros(2,2);
    Spur_p.params.cd=cdP;
    Seva_p.uMat = Spur_p.uMat;
    Seva_p.Jname='J_eva';
    Seva_p.fname='f_dynCD2';
    Seva_p.Jparams.Q=diag(qrTrue(1:4));
    Seva_p.Jparams.Rself=diag(qrTrue(5:6));
    Seva_p.Jparams.Ropp=zeros(2,2);
    Seva_p.params.cd=cdE;
    % Propagate to next time step
    [up,ue,flag,tmp1,tmp2]=f_dyn(Spur_p,Seva_p,gameState_p,zeros(4,1));
    if flag==0
        upp=randsample(1:length(Spur_p.uMat),1,true,up);
        uPurTrue=Spur_p.uMat{upp}(:,1);
        uEvaEst=zeros(gameState_p.nu,gameState_p.kMax);
        for ik=1:length(Seva_p.uMat)
            uEvaEst=uEvaEst+ue(ik)*Seva_p.uMat{ik}(:,1);
        end
        QinflatePur=Qnoisepur+blkdiag(zer2,1*eye2);
        scaleS{n}(1)=uvec(upp);
    else
        uPurTrue=up;
        uEvaEst=ue;
        scaleS{n}(1)=uvec(tmp1);
    end
    if testNN
        qqrr=[diag(Qpur); diag(Qeva); diag(Rpur); diag(Reva);cdP;cdE];
        ministate=xPurMean;
        uStack=predict(network1,[xTrue;qqrr]);
        uStack=double(uStack);
        uPn=uStack(1:2)';
        uEn=uStack(3:4)';
    end
    
    %Omniscient evader
    gameState_e.xPur=xTrue(1:4);
    gameState_e.xEva=xTrue(5:8);
    gameState_e.dt=tstep;
    gameState_e.kMax=1;
    gameState_e.nu=2;
    Spur_e.uMat={0}; Seva_e.uMat={0};
    uhat=unit_vector(vmRGVO_tune(xTrue(1:4),xTrue(5:8),upmax,2,tstep,uEvaBestPerformanceEstimate,safeDecelParamVM));
    for ik=1:length(uvec)
        Spur_e.uMat{ik}=upmax*uvec(ik)*uhat;
    end
%     for ik=1:length(utemp)
%         Spur_e.uMat{ik}=utemp(:,ik);
%     end
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
    [up,ue,flag,tmp1,tmp2]=f_dyn(Spur_e,Seva_e,gameState_e,zeros(4,1));
    if flag==0
        uee=randsample(1:length(Seva_e.uMat),1,true,ue);
        uEvaTrue=Seva_e.uMat{uee}(:,1);
        uPurEst=zeros(gameState_p.nu,gameState_p.kMax);
        for ik=1:length(Spur_e.uMat)
            uPurEst=uPurEst+up(ik)*Spur_e.uMat{ik}(:,1);
        end
        scaleS{n}(2)=uvec(uee);
        QinflateEva=Qnoiseeva+blkdiag(1*eye2, zer2);
    else
        uEvaTrue=ue;
        uPurEst=up;
        scaleS{n}(2)=uvec(tmp2);
    end
    
    xTrue(1:4)=f_dynPur(xTrue(1:4),uPurTrue(:,1),tstep,zeros(2,1));
    xTrue(5:8)=f_dynEva(xTrue(5:8),uEvaTrue(:,1),tstep,zeros(2,1));
    
    if plotFlag==1
    figure(1)
    pause(.1)
    delete(f1); delete(f2)
    f1=scatter(xTrue(1),xTrue(2),'b');
    hold on
    f2=scatter(xTrue(5),xTrue(6),'r');
    axis(axisveck)
    end
    
    xTrueS{n+1}=xTrue;
    upS{n}=uPurTrue(:,1);
    ueS{n}=uEvaTrue(:,1);
    tThisStep=toc
    
end

saving_this_iteration=true
qrS_S{iteriter,1}=qrTrueAll;
xtrueS_S{iteriter}=xTrueS;
upS_S{iteriter}=upS;
ueS_S{iteriter}=ueS;
scaleS_S{iteriter}=scaleS;
end


