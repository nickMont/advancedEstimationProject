%Scratch plotting code

% figure(1);clf;
% figset
% shapes={'-o','-x','-*','-s'};
% for ij=1:nPur
%     xy=[];
%     for ik=1:n+1
%         xtemp=xTrueS{ik}{1}{ij};
%         xy=[xy xtemp(1:2)];
%     end
%     xy
%     plot(xy(1,:),xy(2,:),shapes{ij});
%     hold on
% end
% shapes={':o',':x',':*',':s'};
% for ij=1:nEva
%     xy=[];
%     for ik=1:n+1
%         xtemp=xTrueS{ik}{2}{ij};
%         xy=[xy xtemp(1:2)];
%     end
%     xy
%     plot(xy(1,:),xy(2,:),shapes{ij});
%     hold on
% end
% xlabel('x-position (km)')
% ylabel('y-position (km)')
% title('Trajectories of attacker and defender UAVs')
% legend('D_1','D_2','D_3','A_1','A_2','A_3')
% figset

% % %PE P2020 heuristics
% figure(2);clf;
% figset
% shrink=heurstore(4,:)./heurstore(3,:);
% plot(1:length(heurstore(1,:)),heurstore(1,:),'-x')
% hold on
% plot(1:length(heurstore(2,:)),heurstore(2,:),'-*')
% hold on
% axis([1 length(heurstore) 0 max(heurstore(2,:))+10])
% xlabel('Iteration count')
% ylabel('Execution Time (s)')
% yyaxis right
% plot(1:length(shrink),shrink,'-o')
% set(gca,'ycolor','k')
% ylabel('Control evaluations (fraction of non-heuristic case)')
% legend('Execution time without heuristic','Execution time with heuristic','Reduction in scale')
% figset

% %PES paper
% figure(2);clf;
% figset
% subplot(3,1,1);
% plot(1:n+1,dJS(1,:),'-.k');
% hold on
% plot(1:n+1,dJS(2,:),'-ok');
% ax=gca;
% axis([0 30 ax.YLim(1) ax.YLim(2)])
% legend('\DeltaQ_{xx}','\DeltaQ_{yy}')
% xlabel('Time (s)');
% ylabel('Cost (m^{-2})');
% figset
% 
% subplot(3,1,2);
% figset
% plot(1:n+1,dJS(3,:),'-.k');
% hold on
% plot(1:n+1,dJS(4,:),'-ok');
% ax=gca;
% axis([0 30 ax.YLim(1) ax.YLim(2)])
% legend('\DeltaQ_{vx}','\DeltaQ_{vy}');
% xlabel('Time (s)');
% ylabel('Cost (m^{-2}s^2)');
% figset
% 
% subplot(3,1,3);
% figset
% plot(1:n+1,dJS(5,:),'-.k');
% hold on
% plot(1:n+1,dJS(6,:),'-ok');
% ax=gca;
% axis([0 30 ax.YLim(1) ax.YLim(2)])
% legend('\DeltaR_{x}','\DeltaR_{y}');
% xlabel('Time (s)');
% ylabel('Cost (m^{-2}s^4)');
% figset
% 
% xP=zeros(2,n+1);xE=zeros(2,n+1);
% for ijk=1:n+1
%     xP(:,ijk)=xTrueS{ijk}(1:2); xE(:,ijk)=xTrueS{ijk}(5:6);
% end
% figure(3);clf;
% figset
% plot(xP(1,:),xP(2,:),'-xk');
% hold on
% plot(xE(1,:),xE(2,:),'-ok');
% xlabel('East displacement (m)');
% ylabel('North displacement (m)');
% lgd=legend('Pursuer','Evader');
% lgd.Location ='northeast';
% figset

% %     load quaddat.mat;
% load figs_and_mats\figs_mats_PES\linCompare.mat
% load figs_and_mats\figs_mats_PES\quaddatPES.mat
% indsamp=1:5:100;
% %     xP2d=xStore(7:8,indsamp);
% %     xE2d=xStore(19:20,indsamp);
% mx=(xP2d(2,1)-xE2d(2,1))/(xP2d(1,1)-xE2d(1,1));
% xPlin_x=-5:0.3:1;
% xPlin_y=xPlin_x*mx;
% xElin_x=-6:0.3:0;
% xElin_y=xElin_x*mx;
% 
% figure(1);clf;
% figset
% plot(xP2d(1,:),xP2d(2,:),'-*k');
% hold on
% plot(xE2d(1,:),xE2d(2,:),'-ok');
% hold on
% plot(xPlinCompare(1,:),xPlinCompare(2,:),'-.xk');
% hold on
% plot(xElinCompare(1,:),xElinCompare(2,:),'-.dk');
% axis([-6 2 -6 2])
% xlabel('x-position (m)')
% ylabel('y-position (m)')
% legend('Pursuer, quad','Evader, quad','Pursuer, PM','Evader, PM','Location','southeast');
% figset




% %<<>> clean this up
% figset
% figure(2);clf;
% plottype=['k-x','k-*','k-0'];
% plot(dt*(0:34),muHist(1,1:35),'k-x')
% hold on
% plot(dt*(0:34),muHist(2,1:35),'k-*')
% hold on
% plot(dt*(0:34),muHist(5,1:35),'k-s')
% hold on
% plot(dt*(0:34),muHist(8,1:35),'k-o')
% hold on
% plot(dt*(0:34),sum(muHist([3 4 6 7 9:12],1:35))/8,'k-^')
% % if nmod>=7
% % plot(dt*(0:34),muHist(7,1:35),'k-v')
% % end
% figset
% xlabel('Time Elapsed (s)')
% ylabel('Model Probability')
% legend('VM/GT-1','GT-1','VMGTHeur-1','GT-2','other')
% figset
% %legend('Nash strategy','Non-Nash strategy') %update per side

figure(1);clf;
figset
plot(times,vecnorm(uPhistMeanRotor),'-k.')
hold on
plot(times,vecnorm(uPhistMeanOutput),'-kx')
plot(times,vecnorm(uEhistMeanRotor),'-ko')
plot(times,vecnorm(uEhistMeanOutput),'-k^')
legend('uP,u-space','uP,x-space','uE,u-space','uE,x-space','location','southeast')
xlabel('Time (s)')
ylabel('\|u\| (rad/s)')
title('Control magnitudes for each response algorithm')
figset



