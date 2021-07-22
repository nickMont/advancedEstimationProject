clear;clc;loadenv;

% Paper test params
L = 0.5; %capture range
phi = 3*pi/8; %final capture angle
mu = 0.75; %speed ratio, vmaxE/vmaxP

% sim params
fprintf('Using different params from first paper plot set')
L = 0.5; %capture range
phi = 3*pi/8; %final capture angle
mu = 0.1; %speed ratio, vmaxE/vmaxP

% %P control type
controlTypeP = 'disc'; %'disc','true'
% controlTypeP = 'true'; %'disc','true'

% %E control type
controlTypeE = 'disc'; %'disc','true'
% controlTypeE = 'true'; %'disc','true'



%xBphi = (2*sin(phi)+2*mu*(pi-phi)-L)*sin(phi);
%yBphi = -(2*sin(phi)+2*mu*(pi-phi)-L)*cos(phi);
xBphi = L*sin(phi);
yBphi = L*cos(phi);

a=1; %a>0
lambdaX_T = -a*sin(phi);
lambdaY_T = -a*cos(phi);
% apprx for consistency with inverse time equations (see Pachter&Coates
%  2018)
lambdaDotX_T = -a*cos(phi);
lambdaDotY_T = a*sin(phi);

Tmax = 2*(pi-phi);

params.mu = mu;
params.phi = phi;
stateT = [xBphi; yBphi; lambdaX_T; lambdaY_T; lambdaDotX_T; lambdaDotY_T];
[tOut,stateOut] = ode45(@(t,x) homicidalChauffeurInverseTime(t,x,params), [0 Tmax], stateT);
Tau = tOut;

x0=stateOut(end,1:2)';

% figure(1);clf;
% plot(stateOut(:,1),stateOut(:,2))

lambdaX = flip(stateOut(:,3));
lambdaY = flip(stateOut(:,4));
xHist = flip(stateOut(:,1));
yHist = flip(stateOut(:,2));
tHist = flip(Tmax-tOut);

lambdaM = sqrt(lambdaX.^2+lambdaY.^2);
Psi = atan2(-lambdaX./lambdaM,-lambdaY./lambdaM);
PsiTrueMat = polyfit(tHist,Psi,1);
lambdaTrueMat = [tHist lambdaM];
% Psi = unwrap(Psi);
% figure(2);clf;
% subplot(2,1,1)
% plot(tHist,Psi)
% title('Evader control')

% S = lambdaY.*xHist - lambdaX.*yHist;
% uS = sign(S);
% figure(2);subplot(2,1,2)
% plot(tHist,uS)
% axis([0 Tmax -1.1 1.1])
% title('Pursuer control')

% params2.mu = mu;
% params2.PsiMat = [tHist Psi];
% params2.PsiMat = polyfit(tHist,Psi,1);
% params2.PsiType = 'linear';
% params2.uPMat = [tHist uS;1000 uS(end)];
% foh=@(t,x) homicidalChauffeurForwardFOH(t,x,params2);
% [a,b]=ode45(foh,[0 Tmax],x0);
% figure(4);clf;
% plot(b(:,1),b(:,2))

% a = 9001;
% lambdaXCalcT = -a*sin(phi+Tau);
% lambdaYCalcT = -a*cos(phi+Tau);
% lambdaXCalct = flip(lambdaXCalcT);
% lambdaYCalct = flip(lambdaYCalcT);
% xT = 1+(L-mu*Tau).*sin(phi+Tau)-cos(Tau);
% yT = (L-mu*Tau).*cos(phi+Tau)+sin(Tau);
% xt = flip(xT);
% yt = flip(yT);
% St = sign(xt.*lambdaYCalct - yt.*lambdaXCalct);
% Psit = atan2(-lambdaXCalct,-lambdaYCalct);
% Psit = unwrap(Psit);
% figure(5);clf;
% plot(xT,yT)

% params3.mu = mu;
% params3.PsiMat = [tHist Psit];
% params3.PsiMat = polyfit(tHist,Psit,1)
% params3.PsiType = 'linear'
% params3.uPMat = [tHist St;1000 St(end)];
% foh2=@(t,x) homicidalChauffeurForwardFOH(t,x,params3);
% [a,b]=ode45(foh2,[0 Tmax],x0)
% figure(6);clf;
% plot(b(:,1),b(:,2))


% generate possible u set
uSet=[-1 1];
uCombs = allcomb(uSet,uSet); %0-1 transition
[nu1,~] = size(uCombs);

% Worst-case scenario is interception at -pi
TmaxPos = 2*pi;

%generate transition time
dt = pi;
tTransitionSet = (0+dt):dt:TmaxPos;
uCombs2 = allcomb(1:nu1,tTransitionSet);
[nu2,~]=size(uCombs2);

uPSet={};
n=0;
% for ij=1:nu2
%     if max(diff(uCombs(uCombs2(ij,1),:)))~=0 %if there is a transition    
%         n=n+1;
%         uPSet{n,1}=[-1e-10 0 %smoother handling of t==0 case
%             0 uCombs(uCombs2(ij,1),1)
%             uCombs2(ij,2) uCombs(uCombs2(ij,1),2)];
%     end
% end
% usedSet=[];
% for ij=1:nu1
%     if max(abs(diff(uCombs(ij,:))))<=1e-6 %if there is not a transition
%         ff = find(abs(usedSet-uCombs(ij,1))<1e-6);
%         if isempty(ff)
%             n=n+1;
%             uPSet{n,1}=[-1e-10 0 %smoother handling of t==0 case
%                 0 uCombs(ij,1)];
%             usedSet=[usedSet uCombs(ij,1)]; %#ok<AGROW>
%         end
%     end
% end


% % get multi-transition form
% transitionMax = 6;
% setMax = de2bi(1:2^(transitionMax+1));
% setMax = 2*setMax-1;
% % Following block handles edge error (final column of 0s)
% [a,b]=size(setMax);
% setMax = setMax(:,1:b-1);
% [a,b]=size(setMax);
% % end edge handling
% tMax2 = 2*pi+0.1;
% tMaxPos = tMax2;
% tSet2 = linspace(0,tMax2,transitionMax+1);
% for ij=1:a
%     uPSet{ij,1}=[-1e-6 0
%         tSet2' setMax(ij,:)'];
% end


% % Singular transition form
% dt1 = 0.01;
% tMax3 = 2*pi+0.01;
% tMaxPos = tMax3;
% timeSet = dt1:dt1:tMax3;
% uPSet{1,1}=[-1e-6 1
%     1e6 0];
% uPSet{2,1}=[-1e-6 -1
%     1e6 0];
% for ij=1:length(timeSet)
%     uPSet{2*ij} = [-1e-6 0
%         0 1
%         timeSet(ij) -1];
%     uPSet{2*ij+1} = [-1e-6 0
%         0 -1
%         timeSet(ij) 1];
% end


nuF=max(size(uPSet));

% % generate Psi set from linear form
% dM=pi/8;
% dB=dM;
% mSet = -pi/2:dM:pi/2;
% bSet = -pi:dB:pi;
% qSet = -0.1:0.05:0.01;
% 
% PsiComb = allcomb(mSet,bSet);
% % PsiComb = allcomb(qSet,mSet,bSet);
% [npF,~]=size(PsiComb);
% 
% PsiSet={};
% for ij=1:npF
%     PsiSet{ij,1}=PsiComb(ij,:); %#ok<SAGROW>
% end


% % Add known solution
% PsiSet{npF+1,1}=params2.PsiMat;
% uPSet{nuF+1,1}=[tHist uS];


% % Params for discretized optimizer
% Spur.uMat = uPSet;
% Seva.uMat = PsiSet;
% gameState.params.uEType = 'linear';
% gameState.params.mu=mu;
% gameState.tMaxSim = TmaxPos;
% gameState.x0=x0;
% gameState.params.L=L;


% tic
% 
% % Run discretized optimizer
% [temp,~,temp2,~,culledset] = generateCostMatricesHC(Spur,Seva,gameState,[]);
% 
% 
% toc
% 
% minAll=minimax1(temp);
% [cullP,cullE]=size(temp2);
% minCull=minimax1(temp2);
% discTime=temp2(minCull(1),minCull(2))
% trueTime=Tmax

dtSim=.5;
dtStep=.1;
transitionMax = 5;

xCurr=x0;

n=0;
stopcond=false;
tInternal=0;
tDstore=[];
xDstore=[];
uDPstore=[];
while stopcond==false
    n=n+1;
    
    %P set
%     % Generate -1/0/1 bang/off/bang
%     setMaxD = dec2base(1:3^(transitionMax+1),3);
%     [a,~]=size(setMaxD);
%     setMax=[];
%     for ij=1:a
%         rr=setMaxD(ij,:);
%         b=length(rr);
%         r2=[];
%         for ik=1:b
%             r2=[r2 str2num(rr(ik))];
%         end
%         setMax=[setMax; r2];
%     end
%     setMax=setMax-1;
    
    % Generate -1/1 bangbang
    setMax = de2bi(1:2^(transitionMax+1));
    setMax = 2*setMax-1;

    % Following block handles edge error (final column of 0s)
    [~,b]=size(setMax);
    setMax = setMax(:,1:b-1);
    [a,~]=size(setMax);
    % End edge handling
    
    % Convert setMax into usable controls
    tMax2 = tInternal+dtStep*transitionMax;
    tMaxPos = tMax2;
    tSet2 = linspace(tInternal,tMax2,transitionMax+1);
    for ij=1:a
        tm=[-1e-6 0
            tSet2' setMax(ij,:)'
            tInternal+dtStep*transitionMax+.1 0];
        uPSet{ij,1}=tm;
    end
    
    % E set
    dM=pi/8;
    dB=dM;
    mSet = -pi/2:dM:pi/2;
    bSet = -pi:dB:pi;
    qSet = -0.1:0.05:0.01;
    PsiComb = allcomb(mSet,bSet);
    % PsiComb = allcomb(qSet,mSet,bSet);
    [npF,~]=size(PsiComb);
    PsiSet={};
    for ij=1:npF
        PsiSet{ij,1}=PsiComb(ij,:); %#ok<SAGROW>
    end
    
%     fprintf('TEMP SETTING PSI TO 0 ONLY')
%     PsiSet={};
%     PsiSet{1,1}=[0 0];
    
    % fill optimizer mats
    Spur.uMat = uPSet;
    Seva.uMat = PsiSet;
    
    gameState.params.uEType = 'linear';
    gameState.params.mu=mu;
    gameState.t0=tInternal;
    gameState.tMaxSim = tInternal+transitionMax*dtStep;
    gameState.x0=xCurr;
    gameState.params.L=L;
    
    %run optimizer
    if strcmp(controlTypeP,'disc') || strcmp(controlTypeE,'disc')
        [temp,~,temp2,~,culledset] = generateCostMatricesHC_closedloop(Spur,Seva,gameState,[]);
        minAll=minimax1(temp);
        [cullP,cullE]=size(temp2);
        minCull=minimax1(temp2);
    end
    
    % propagate
    if strcmp(controlTypeP,'disc')
        uPt=Spur.uMat{minAll(1)};
    elseif strcmp(controlTypeP,'true')
        uPt = lambdaTrueMat;
        uPt = [uPt; 1000 uPt(end,2)];
    else
        error('Unrecognized pursuer control type')
    end
        
    if strcmp(controlTypeE,'disc')
        uEt=Seva.uMat{minAll(2)};
    elseif strcmp(controlTypeE,'true')
        uEt=PsiTrueMat;
    else
        error('Unrecognized pursuer control type')
    end
        
    uPrun = uPt
    uErun = uEt
    
    
    hc_params.mu = gameState.params.mu;
    hc_params.PsiMat = uEt;
    hc_params.uPMat = uPt;
    hc_params.PsiType = gameState.params.uEType;
    HC_ODE = @(t,x) homicidalChauffeurForwardFOH(t,x,hc_params);
    [aa,bb]=ode45(HC_ODE,[tInternal tInternal+dtSim],xCurr);
    xCurr=bb(end,:)';
    
    tDstore=[tDstore;aa];
    xDstore=[xDstore;bb];
    
    tlow=find((uPt(:,1)>tInternal-0.01));
    thigh=find((uPt(:,1)-.01<tInternal+dtSim-0.01));
    tadd=intersect(tlow,thigh);
    uDPstore=[uDPstore;
        uPt(tadd,:)];
    
    % termination criteria
    l2 = bb(:,1).^2 + bb(:,2).^2;
    bDot = HC_ODE(a(end),bb(end,:)');
    indf = find(l2<=(gameState.params.L)^2);
    %Does capture occur at indf?
    doesCap=false;
    capTime=9001;
    if length(indf)>=1
        for ij=1:length(indf)
            xydot=HC_ODE(aa(indf(ij)),bb(indf(ij),:)');
            if bb(indf(ij),1)/xydot(1)<0 && bb(indf(ij),2)/xydot(2)<0
                doesCap = true;
                if aa(indf(ij))<capTime
                    capTime=aa(indf(ij));
                end
            end
        end
    end
    
    tInternal=tInternal+dtSim;
    if doesCap || tInternal>2*pi+dtSim || n>1000
        stopcond=true;
    end
end

tTrue = Tmax
tDisc = capTime

figure(1);clf;
plot(xDstore(:,1),xDstore(:,2),'b')
hold on
plot(stateOut(:,1),stateOut(:,2),'r')
title('XY')

figure(2);clf;
plot(tDstore,xDstore(:,1),'b')
hold on
plot(tOut,stateOut(length(tOut):-1:1,1),'r')
title('t,X')

figure(3);clf;
plot(tDstore,xDstore(:,2),'b')
hold on
plot(tOut,stateOut(length(tOut):-1:1,2),'r')
title('t,Y')

figure(4);clf;
stairs(uDPstore(:,1),uDPstore(:,2))
title('uP')

