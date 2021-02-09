clear;clc;loadenv;

%
% Game state convention
%        +x
%         |
%         |
%   +y - -
%

%NOTE: CURRENTLY SET FOR U BASED ON VELOCITY MATCHING SCALE


% rngseedno=2;
% rng(rngseedno)
numNNiter=1;

flagHasTarget=true;
flagPursuerUsesPureVM=true;

qrStore=cell(1,numNNiter);
xStore=cell(1,numNNiter);
measStore=cell(1,numNNiter);
controlTypeStore=cell(1,numNNiter);
uStore=cell(2,numNNiter);
if flagHasTarget==true
    targetStore=cell(2,numNNiter);
end

for ikN=1:numNNiter

currentStep=ikN/numNNiter*100    

ctrTypeR=floor(rand*3);

ctrTypeR=0;
'DO NOT USE THIS CODE UNTIL CTRTYPER IS RANDOM AGAIN'

ctrTypeR=0;

if ctrTypeR==1
    controlType='vmgt';
elseif ctrTypeR==2
    controlType='vm';
else
    controlType='gt';
end
controlTypeStore{ikN}=ctrTypeR;


%load nnvarDynTrained0417.mat
%network1=net;
numRefinementsP=0;
numRefinementsE=0;
uEvaBestPerformanceEstimate=[0;0];
safeDecelParamVM=0.5;
    
plotFlag=0;
plotEndFlag=0;

tstep=0.5;
tmax=20;

%utemp=permn(-2:0.2:2,2)';
uvec=-1:0.1:1;
utemp=permn(uvec,2)';
upmax=1;
umax=upmax;
utype.radius=upmax;
utemp=mapUtempToUvec(utemp,"circle",utype);

Game.uType="velstep";
%Options: velstep (step of constant size, from velocity)
%         accel (double integrator)
Game.dt=tstep;

%xTrue=[[0 0 0 0.5]'; [2 10 0 0.2]'];
%xTrue = diag([20 20 2 2 20 20 2 2])*(2*rand(8,1)-1);
xTrue = diag([4 4 .5 .5 4 4 .5 .5])*rand(8,1);

xTarget=20*(rand(2,1)-0.5);

Qpur = 100*rand*diag([1 1 0 0]); Rpur = 15*rand*diag([1 1]);
Qeva = 100*rand*diag([1 1 0 0]); Reva = 15*rand*diag([1 1]);
QTargetPur=100*rand*diag([1 1]);
QTargetEva=100*rand*diag([1 1]);
% Qpur = diag(100*rand(1,4)); Rpur = diag(100*rand(1,2));
% Qeva = diag(100*rand(1,4)); Reva = diag(100*rand(1,2));

qrTrue=[diag(Qeva);diag(Reva)];
cdP=.1;
cdE=.1;
qrTrueAll=[diag(Qpur); diag(Rpur); diag(Qeva); diag(Reva);cdP;cdE];
if flagHasTarget
    qrTrueAll=[qrTrueAll; diag(QTargetPur); diag(QTargetEva); xTarget];
end
qrStore{ikN}=qrTrueAll;

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
QnoiseStack=blkdiag(Qnoisepur,Qnoiseeva);
GQG=Gnoiseeva*Qnoiseeva*Gnoiseeva';
cholQ2p=chol(Qnoisepur(1:2,1:2))'; cholQ2e=chol(Qnoisepur(3:4,3:4))';
Bnoiseeva=Gnoiseeva;
Peva=1e-2*eye(8);

Ppur=Peva;

%This can be shunted off to a separate function
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

n=0;
xTrueS={};
xTrueS{1}=xTrue;
upS=cell(floor(tmax/tstep),1);ueS=cell(floor(tmax/tstep),1);
scaleS=cell(floor(tmax/tstep),1);
zS={};

Jp0=0;
Je0=0;
uBp=[];
uBe=[];
xBt=xTrue;
zB=[];

heurTypeStruc{1}='gt';
heurTypeStruc{2}='vmgt';
heurTypeStruc{3}='vm';
nmod=length(heurTypeStruc);
numRefinements=0;
xEva=xTrue(5:8);
xhatE=repmat(xEva,[1 nmod]);
PhatE=repmat(0.001*eye(length(xEva)),[1 1 nmod]);
mu=1/(nmod)*ones(nmod,1);
muHist=mu;
Rk=0.1*eye(length(xEva));

for ij=0:tstep:tmax
    n=n+1;
    tic
    
    %reset noise inflations
    QinflatePur=Qnoisepur;
    QinflateEva=Qnoiseeva;
    scaleS{n}=zeros(2,1);
    
    xPurMean=[xPur;diag(Qeva);diag(Reva)];
    xEva=xEva;
    %    xPurMean=xTrue+1e-2*randn(8,1);
    %    xEva=xTrue+1e-2*randn(8,1);
    
    uEvaVM=uEvaBestPerformanceEstimate;
    for im=1:numRefinementsP+1
        % Guessing pursuer
        Spur_p.uMat={0};
        Seva_p.uMat={0};
        gameState_p.xPur=xTrue(1:4);
        gameState_p.xEva=xTrue(5:8);
        gameState_p.dt=tstep;
        gameState_p.kMax=1;
        gameState_p.nu=2;
        uhat=vmRGVO_tune(xTrue(1:4),xTrue(5:8),upmax,2,tstep,uEvaVM,safeDecelParamVM);
        if flagHasTarget
            dxe=xTrue(1:2)-xTrue(5:6);
            dxt=xTarget-xTrue(5:6);
            uhat=uhat*(dxe'*Qeva(1:2,1:2)*dxe)+dxt*(dxt'*QTargetEva*dxt);
        end
        uhat=unit_vector(uhat);
        if strcmp(controlType,'vmgt')
            for ik=1:length(uvec)
                Spur_p.uMat{ik}=upmax*uvec(ik)*uhat;
            end
        elseif strcmp(controlType,'gt')
            for ik=1:length(utemp)
                Spur_p.uMat{ik}=utemp(:,ik);
            end
        elseif strcmp(controlType,'vm')
            Spur_p.uMat{1}=uhat*upmax*0.8;
        else
            error('Invalid control type')
        end
        Spur_p.Jname='J_pur';
        Spur_p.fname='f_dynPur';
        Spur_p.Jparams.Q=Qpur;
        Spur_p.Jparams.Rself=Rpur;
        Spur_p.Jparams.Ropp=zeros(2,2);
        Spur_p.params.cd=cdP;
        Seva_p.uMat = Spur_p.uMat;
        Seva_p.Jname='J_eva';
        Seva_p.fname='f_dynEva';
        Seva_p.Jparams.Q=diag(qrTrue(1:4));
        Seva_p.Jparams.Rself=diag(qrTrue(5:6));
        Seva_p.Jparams.Ropp=zeros(2,2);
        Seva_p.params.cd=cdE;
        if flagHasTarget
            Spur_p.Jname='J_purTarget';
            Spur_p.Jparams.Q_target=QTargetPur;
            Spur_p.Jparams.x_target=xTarget;
            Seva_p.Jname='J_evaTarget';
            Seva_p.Jparams.Q_target=QTargetEva;
            Seva_p.Jparams.x_target=xTarget;
        end
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
            %scaleS{n}(1)=uvec(upp);
        else
            uPurTrue=up;
            uEvaEst=ue;
            %scaleS{n}(1)=uvec(tmp1);
        end
        if flag==0
            uee=randsample(1:length(Seva_e.uMat),1,true,ue);
            uEvaTrue=Seva_e.uMat{uee}(:,1);
            uPurEst=zeros(gameState_p.nu,gameState_p.kMax);
            for ik=1:length(Spur_e.uMat)
                uPurEst=uPurEst+up(ik)*Spur_e.uMat{ik}(:,1);
            end
            %scaleS{n}(2)=uvec(uee);
            QinflateEva=Qnoiseeva+blkdiag(1*eye2, zer2);
        else
            uEvaTrue=ue;
            uPurEst=up;
            %scaleS{n}(2)=uvec(tmp2);
        end
        uEvaVM=uEvaEst;
    end
    
    Seva_e=Seva_p;
    Spur_e=Spur_p;

    if flagPursuerUsesPureVM
        uhat=vmRGVO_tune(xTrue(1:4),xTrue(5:8),upmax,2,tstep,uEvaVM,safeDecelParamVM);
        if flagHasTarget
            dxe=xTrue(1:2)-xTrue(5:6);
            dxt=xTarget-xTrue(5:6);
            uhat=uhat*(dxe'*Qeva(1:2,1:2)*dxe)+dxt*(dxt'*QTargetEva*dxt);
        end
        uhat=unit_vector(uhat);
        uPurTrue=uhat*upmax;
%         error('inc')
    end

    xTrue(1:4)=f_dynCD2(xTrue(1:4),uPurTrue(:,1),tstep,zeros(2,1),Spur_p.params);
    xTrue(5:8)=f_dynCD2(xTrue(5:8),uEvaTrue(:,1),tstep,zeros(2,1),Seva_e.params);

    z=xTrue+chol(QnoiseStack)'*rand(8,1);

    e=xTrue(1:4)-xTrue(5:8);
    Jlocp = e'*Spur_p.Jparams.Q*e + uPurTrue'*Spur_p.Jparams.Rself*uPurTrue + ...
        uEvaTrue'*Spur_p.Jparams.Ropp*uEvaTrue;
    Jloce = -e'*Seva_e.Jparams.Q*e + uEvaTrue'*Seva_e.Jparams.Rself*uEvaTrue + ...
        uPurTrue'*Seva_e.Jparams.Ropp*uPurTrue;
    Jp0=Jp0+Jlocp;
    Je0=Je0+Jloce;
    
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
    zS{n}=z;
    tThisStep=toc;
    
    xBt=[xBt xTrue];
    uBp=[uBp uPurTrue(:,1)];
    uBe=[uBe uEvaTrue(:,1)];
    zB=[zB z];
    for iN=1:3
        for im=1:numRefinements+1
            controlType=heurTypeStruc{iN};
            Spur_p.uMat={0}; Seva_p.uMat={0};
            
            gameState_p.xPur=xTrue(1:4);
            gameState_p.xEva=xTrue(5:8);
            gameState_p.dt=tstep;
            gameState_p.kMax=1;
            gameState_p.nu=2;
            uhat=vmRGVO_tune(xTrue(1:4),xTrue(5:8),upmax,2,tstep,uEvaVM,safeDecelParamVM);
            if flagHasTarget
                dxe=xTrue(1:2)-xTrue(5:6);
                dxt=xTarget-xTrue(5:6);
                uhat=uhat*(dxe'*Qeva(1:2,1:2)*dxe)+dxt*(dxt'*QTargetEva*dxt);
            end
            uhat=unit_vector(uhat);
            if strcmp(controlType,'vmgt')
                for ik=1:length(uvec)
                    Spur_p.uMat{ik}=upmax*uvec(ik)*uhat;
                end
            elseif strcmp(controlType,'gt')
                for ik=1:length(utemp)
                    Spur_p.uMat{ik}=utemp(:,ik);
                end
            elseif strcmp(controlType,'vm')
                Spur_p.uMat{1}=uhat*upmax*0.8;
            else
                error('Invalid control type')
            end
            Spur_p.Jname='J_pur';
            Spur_p.fname='f_dynPur';
            Spur_p.Jparams.Q=Qpur;
            Spur_p.Jparams.Rself=Rpur;
            Spur_p.Jparams.Ropp=zeros(2,2);
            Spur_p.params.cd=cdP;
            Seva_p.uMat = Spur_p.uMat;
            Seva_p.Jname='J_eva';
            Seva_p.fname='f_dynEva';
            Seva_p.Jparams.Q=diag(qrTrue(1:4));
            Seva_p.Jparams.Rself=diag(qrTrue(5:6));
            Seva_p.Jparams.Ropp=zeros(2,2);
            Seva_p.params.cd=cdE;
            if flagHasTarget
                Spur_p.Jname='J_purTarget';
                Spur_p.Jparams.Q_target=QTargetPur;
                Spur_p.Jparams.x_target=xTarget;
                Seva_p.Jname='J_evaTarget';
                Seva_p.Jparams.Q_target=QTargetEva;
                Seva_p.Jparams.x_target=xTarget;
            end
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
                %scaleS{n}(1)=uvec(upp);
            else
                uPurTrue=up;
                uEvaEst=ue;
                %scaleS{n}(1)=uvec(tmp1);
            end
            if flag==0
                uee=randsample(1:length(Seva_e.uMat),1,true,ue);
                uEvaTrue=Seva_e.uMat{uee}(:,1);
                uPurEst=zeros(gameState_p.nu,gameState_p.kMax);
                for ik=1:length(Spur_e.uMat)
                    uPurEst=uPurEst+up(ik)*Spur_e.uMat{ik}(:,1);
                end
                %scaleS{n}(2)=uvec(uee);
                QinflateEva=Qnoiseeva+blkdiag(1*eye2, zer2);
            else
                uEvaTrue=ue;
                uPurEst=up;
                %scaleS{n}(2)=uvec(tmp2);
            end
            uEvaVM=uEvaEst;
        end
        if iN==1 %VMGT
            uPur1=uPurEst;
            uEva1=uEvaTrue;
            controlType=heurTypeStruc{2};
            RuStack{1}=.05*eye(2);
        elseif iN==2 %GT
            uPur2=uPurEst;
            uEva2=uEvaTrue;
            controlType=heurTypeStruc{3};
            RuStack{2}=.05*eye(2);
        elseif iN==3 %VM
            uPur3=uPurEst;
            uEva3=uEvaTrue;
            controlType=heurTypeStruc{1};
            RuStack{3}=.2*eye(2);
        else
            error('iN error');
        end
        uEvaTempStack{iN}=uEvaTrue;
%         uIN=uEvaTrue
    end
        % "measure"
    z=xTrue+chol(QnoiseStack)'*rand(8,1);
    zMeas=z(5:8);
    
    lambdaTemp=-1*ones(nmod,1);
    for ij=1:nmod
        uEvaMMF=uEvaTempStack{ij};
        RuMMF=RuStack{ij};
        
        Paug=PhatE(:,:,ij); Qk=RuMMF;
        if min(eig(Paug))>0 %check for impossible models
            % Propagate
            [xhatEp1,Pp1]=ukfPropagate(xhatE(:,ij),PhatE(:,:,ij),uEvaMMF,RuMMF,dt,'f_dynEva');
            
            % Measure
            [xhatE(:,ij),PhatE(:,:,ij),nu,Sk,Wk]=kfMeasure(xhatEp1,Pp1,zMeas,eye(length(xEva)),Rk);
            Skt=triu(Sk); Skt=Skt'+Skt; Skt(1:5:end)=diag(Sk); %forces symmetry
            normpEval= mvnpdf(nu,zeros(length(xEva),1),Skt);
        else
            normpEval=1e-20;
        end
        if normpEval<=1e-5
            normpdf_eval=1e-5;
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
    mu=muTemp/sum(muTemp)
    muHist=[muHist mu];
    
    load nnTrainSets\nnDetermineControlType\pointMassTargetPurVM\network1.mat
    classify(net,qrXt)
    
    figure(1);clf; figset
    hold on
    plot(1:length(muHist),muHist(1,:),'r')
    hold on
    plot(1:length(muHist),muHist(2,:),'b')
    hold on
    plot(1:length(muHist),muHist(3,:),'g')
    figset
    
end


xStore{ikN}=xBt;
uStore{1,ikN}=uBp;
uStore{2,ikN}=uBe;
measStore{ikN}=zB;


if plotEndFlag==1
    xP=zeros(2,n+1);xE=zeros(2,n+1);
    for ijk=1:n+1
        xP(:,ijk)=xTrueS{ijk}(1:2); xE(:,ijk)=xTrueS{ijk}(5:6);
    end
    figure(3);clf;
    figset
    plot(xP(1,:),xP(2,:),'-xr');
    hold on
    plot(xE(1,:),xE(2,:),'-ob');
    title('Interceptor using velocity matching vs unaware evader');
    xlabel('East displacement (m)');
    ylabel('North displacement (m)');
    legend('Pursuer','Evader','Location','Southeast');
%     axis(axisveck)
    figset
    
    figure(4); clf;
    subplot(2,1,1)
    figset
    plot(0:tstep:tmax+tstep,xP(1,:),'-xr')
    hold on
    plot(0:tstep:tmax+tstep,xE(1,:),'-ob')
    xlabel('Time (s)')
    ylabel('East displacement (m)')
    legend('Pursuer','Evader','Location','Southeast');
    figset
    subplot(2,1,2)
    figset
    plot(0:tstep:tmax+tstep,xP(2,:),'-xr')
    hold on
    plot(0:tstep:tmax+tstep,xE(2,:),'-ob')
    xlabel('Time (s)')
    ylabel('East displacement (m)')
    legend('Pursuer','Evader','Location','Southeast');
    figset
    
end


end

flagUsedVMforPur=flagPursuerUsesPureVM