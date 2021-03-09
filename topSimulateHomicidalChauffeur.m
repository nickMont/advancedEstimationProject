clear;clc;loadenv;

L = 0.5; %capture range
phi = pi/4; %final capture angle
mu = 0.5; %speed ratio, vmaxE/vmaxP
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
Psi = unwrap(Psi);
% figure(2);clf;
% plot(tHist,Psi)

S = lambdaY.*xHist - lambdaX.*yHist;
uS = sign(S);
% figure(3);clf;
% plot(tHist,uS)

params2.mu = mu;
params2.PsiMat = [tHist Psi];
params2.PsiMat = polyfit(tHist,Psi,1);
params2.PsiType = 'linear';
params2.uPMat = [tHist uS;1000 uS(end)];
foh=@(t,x) homicidalChauffeurForwardFOH(t,x,params2);
[a,b]=ode45(foh,[0 Tmax],x0);
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
% 
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
dt = 0.1;
tTransitionSet = (0+dt):dt:TmaxPos;
uCombs2 = allcomb(1:nu1,tTransitionSet);
[nu2,~]=size(uCombs2);

uPSet={};
n=0;
for ij=1:nu2
    if max(diff(uCombs(uCombs2(ij,1),:)))~=0 %if there is a transition    
        n=n+1;
        uPSet{n,1}=[-1e-10 0 %smoother handling of t==0 case
            0 uCombs(uCombs2(ij,1),1)
            uCombs2(ij,2) uCombs(uCombs2(ij,1),2)];
    end
end
usedSet=[];
for ij=1:nu1
    if max(abs(diff(uCombs(ij,:))))<=1e-6 %if there is not a transition
        ff = find(abs(usedSet-uCombs(ij,1))<1e-6);
        if isempty(ff)
            n=n+1;
            uPSet{n,1}=[-1e-10 0 %smoother handling of t==0 case
                0 uCombs(ij,1)];
            usedSet=[usedSet uCombs(ij,1)]; %#ok<AGROW>
        end
    end
end

nuF=max(size(uPSet));

% generate Psi set from linear form
mSet = -1:0.1:1;
bSet = -1:0.1:1;

PsiComb = allcomb(mSet,bSet);
[npF,~]=size(PsiComb);

PsiSet={};
for ij=1:npF
    PsiSet{ij,1}=PsiComb(ij,:); %#ok<SAGROW>
end



% % Add known solution
% PsiSet{npF+1,1}=params2.PsiMat;
% uPSet{nuF+1,1}=[tHist uS];



% Params for discretized optimizer
Spur.uMat = uPSet;
Seva.uMat = PsiSet;
gameState.params.uEType = 'linear';
gameState.params.mu=mu;
gameState.tMaxSim = TmaxPos;
gameState.x0=x0;
gameState.params.L=L;

% Run discretized optimizer
[temp,~,temp2,~,culledset] = generateCostMatricesHC(Spur,Seva,gameState,[]);

minAll=minimax1(temp);
[cullP,cullE]=size(temp2);
minCull=minimax1(temp2);




