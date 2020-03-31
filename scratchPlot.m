%Scratch plotting code

% figure(1);clf;
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

figure(1);clf;
shrink=heurstore(4,:)./heurstore(3,:);
plot(1:length(heurstore(1,:)),heurstore(1,:),'-x')
hold on
plot(1:length(heurstore(2,:)),heurstore(2,:),'-*')
hold on
axis([1 length(heurstore) 0 max(heurstore(2,:))+10])
xlabel('Iteration count')
ylabel('Execution Time (s)')
yyaxis right
plot(1:length(shrink),shrink,'-o')
ylabel('Control evaluations with heuristic (fraction of non-heuristic case)')
legend('Execution time without heuristic','Execution time with heuristic','Reduction in scale')
figset

