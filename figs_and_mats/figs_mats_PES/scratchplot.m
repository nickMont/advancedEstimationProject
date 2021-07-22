% loadenv

%PES paper
dyAxLim=6;
figure(1);clf;
figset
s1=subplot(3,1,1);
sp1_1=plot(1:n+1,dJS(1,:),'-.k');
hold on
sp1_2=plot(1:n+1,dJS(2,:),'-ok');
ax=gca;
axis([0 30 ax.YLim(1)-dyAxLim ax.YLim(2)+dyAxLim])
legend('\DeltaQ_{xx}','\DeltaQ_{yy}')
ylabel('Cost (m^{-2})');
set(gca,'XTick',[])
figset

s2=subplot(3,1,2);
figset
sp2_1=plot(1:n+1,dJS(3,:),'-.k');
hold on
sp2_2=plot(1:n+1,dJS(4,:),'-ok');
ax=gca;
axis([0 30 ax.YLim(1)-dyAxLim ax.YLim(2)+dyAxLim])
legend('\DeltaQ_{vx}','\DeltaQ_{vy}');
ylabel('Cost (m^{-2}s^2)');
set(gca,'XTick',[])
figset

s3=subplot(3,1,3);
figset
sp3_1=plot(1:n+1,dJS(5,:),'-.k');
hold on
sp3_2=plot(1:n+1,dJS(6,:),'-ok');
ax=gca;
axis([0 30 ax.YLim(1)-dyAxLim ax.YLim(2)+dyAxLim])
legend('\DeltaR_{x}','\DeltaR_{y}');
xlabel('Time (s)');
ylabel('Cost (m^{-2}s^4)');
figset

p1 = get(s1,'Position')
p2 = get(s2,'Position')
p3 = get(s3,'Position')
% p2(2) = p1(2)+p1(4);
nh=.27;
p1t=p1; p1t(4) = nh;
set(s1,'Position',p1t)
p2t=p2; p2t(4) = nh;
set(s2,'Position',p2t)
p3t=p3; p3t(4) = nh;
set(s3,'Position',p3t)


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



