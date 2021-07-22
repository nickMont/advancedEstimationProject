% loadenv;
load hc_14step_3pi8.mat

Psi = unwrap(Psi);
% figure(2);clf;
% subplot(2,1,1)
% plot(tHist,Psi)
% title('Evader control')

S = lambdaY.*xHist - lambdaX.*yHist;
uS = sign(S);
% figure(2);subplot(2,1,2)
% plot(tHist,uS)
% axis([0 Tmax -1.1 1.1])
% title('Pursuer control')

params2.mu = mu;
params2.PsiMat = [tHist Psi];
params2.PsiMat = polyfit(tHist,Psi,1);
params2.PsiType = 'linear';
params2.uPMat = [tHist uS;1000 uS(end)];
foh=@(t,x) homicidalChauffeurForwardFOH(t,x,params2);
[aC,bC]=ode45(foh,[0 Tmax],x0);
% figure(4);clf;
% plot(bC(:,1),bC(:,2))

a = 9001;
lambdaXCalcT = -a*sin(phi+Tau);
lambdaYCalcT = -a*cos(phi+Tau);
lambdaXCalct = flip(lambdaXCalcT);
lambdaYCalct = flip(lambdaYCalcT);
xT = 1+(L-mu*Tau).*sin(phi+Tau)-cos(Tau);
yT = (L-mu*Tau).*cos(phi+Tau)+sin(Tau);
xt = flip(xT);
yt = flip(yT);
St = sign(xt.*lambdaYCalct - yt.*lambdaXCalct);
Psit = atan2(-lambdaXCalct,-lambdaYCalct);
Psit = unwrap(Psit);
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


% Plot true path
params2.mu = mu;
pv = PsiSet{culledset(minCull(2))};
params2.PsiMat = pv;
params2.PsiType = 'linear';
uPD=cell2mat(uPSet(minCull(1)));
uED=pv;
params2.uPMat = [uPD(:,1) uPD(:,2);1000 uS(end)];
foh=@(t,x) homicidalChauffeurForwardFOH(t,x,params2);
[aD,bD]=ode45(foh,[0 Tmax],x0);
% figure(7);clf;
% plot(bD(:,1),bD(:,2))

% % Plot capture radius
% figure(8);clf;
% plot(aD,sqrt(bD(:,1).^2+bD(:,2).^2),'k')
% hold on
% plot([0 max(aD)],[0.5 0.5],'b')
% figure(9);clf;
% plot(aC,sqrt(bC(:,1).^2+bC(:,2).^2),'k')
% hold on
% plot([0 max(aC(1:length(aC)-1))],[0.5 0.5],'b')

% Plot both C and D control
Psi = unwrap(Psi);
uEDplot = uED(1)*uPD(:,1)+uED(2);
S = lambdaY.*xHist - lambdaX.*yHist;
uS = sign(S);
figure(10);clf;s10_1=subplot(2,1,1); figset
plot(tHist,uS,'-*k')
hold on
stairs(uPD(:,1),uPD(:,2),'-ok')
axis([0 discTime -1.1 1.1])
title('Pursuer control')
legend('True solution','Discretized','Location','Southwest')
ylabel('Pursuer control')
set(gca,'XTick',[])
figure(10);figset;s10_2=subplot(2,1,2);figset
plot(tHist,Psi,'-*k')
hold on
plot(uPD(:,1),uEDplot,'-ok')
legend('True solution','Discretized','Location','Southeast')
title('Evader control')
axis([0 discTime min([uEDplot;Psi])-.1 max([uEDplot;Psi])+.1])
ylabel('Evader control')
xlabel('Nondimensionalized Time')
figset
figure(10);
nh=.41;
p1 = get(s10_1,'Position')
p2 = get(s10_2,'Position')
p1t=p1; p1t(4) = nh;
set(s10_1,'Position',p1t)
p2t=p2; p2t(4) = nh; p2t(2)=.15;
set(s10_2,'Position',p2t)

