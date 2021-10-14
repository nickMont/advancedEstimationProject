loadenv;

% OQE plots
muPlot=muHist;
figure(1);clf;figset
timePlot=t0:dt:t+dt;
plot(timePlot,muPlot(1,:),'-*k')
hold on
plot(timePlot,muPlot(2,:),'-+k')
plot(timePlot,muPlot(3,:),'-.k')
plot(timePlot,muPlot(6,:),'-^k')
plot(timePlot,sum(muPlot([4 5 7:end],:)),'-ok')
figset
legend('HQNO 1','QNO 1','QNO/PM 1','HQNO 2','others')
xlabel('Time elapsed (s)')
ylabel('Model probability')
figset


