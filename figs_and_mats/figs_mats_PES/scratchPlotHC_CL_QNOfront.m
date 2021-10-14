clear;clc;loadenv


load('E:\github\advancedEstimationProject\figs_and_mats\figs_mats_PES\HC_QNOCL_vs_QNOCL.mat')
tt=abs((cell2mat(tDiscS)-cell2mat(tTrueS)))./cell2mat(tTrueS);
l2=length(tt);
pi2Ind=find(phiSet>=1.5,1);

figure(1);clf;figset
scatter(phiSet(1:pi2Ind),100*tt(1:pi2Ind),'.k');
figset
xlabel('\phi')
ylabel('Percent deviation from true cost')
axis([0 pi/2 0 15])
title('QNO Pursuer vs QNO Evader')
figset
% title('QNO Pursuer vs QNO Evader','FontName','Times New Roman','FontSize',12);
% t.FontSize=12;
% t.FontName='times new roman';
mean(100*tt(1:pi2Ind))


load('E:\github\advancedEstimationProject\figs_and_mats\figs_mats_PES\HC_QNOCL_vs_true5.mat')
tt=abs((cell2mat(tDiscS)-cell2mat(tTrueS)))./cell2mat(tTrueS);
l2=length(tt);
pi2Ind=find(phiSet>=1.5,1);

figure(2);clf;figset
scatter(phiSet(1:pi2Ind),100*tt(1:pi2Ind),'.k');
figset
xlabel('\phi')
ylabel('Percent deviation from true cost')
axis([0 pi/2 0 15])
title('QNO Pursuer vs Analytical Evader')
figset
% title('QNO Pursuer vs Analytical Evader','FontName','Times New Roman','FontSize',12)
mean(100*tt(1:pi2Ind))





