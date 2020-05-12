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
figset
plot(1:n+1,dJS(3,:),'-.r');
hold on
plot(1:n+1,dJS(4,:),'-ob');
legend('\DeltaQ_{vx}','\DeltaQ_{vy}');
xlabel('Time (s)');
ylabel('Cost parameter (m^{-2}s^2)');
figset

subplot(3,1,3);
figset
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
figset
plot(xP(1,:),xP(2,:),'-xr');
hold on
plot(xE(1,:),xE(2,:),'-ob');
xlabel('East displacement (m)');
ylabel('North displacement (m)');
legend('Pursuer','Evader');
figset
