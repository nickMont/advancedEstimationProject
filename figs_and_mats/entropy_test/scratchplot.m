
% load figs_and_mats\entropy_test\entropyKindaWorks3.mat

dm_en = outmu(:,2)-outmu(:,1);
dm_en_pc = dm_en./outmu(:,1);

dm_dc = outmu(:,3)-outmu(:,1);
dm_dc_pc = dm_en./outmu(:,1);


figure(1);clf;
figset
plot(((1:length(dm_en_pc))-1)*dt,dm_en_pc*100,'ok')
hold on
plot(((1:length(dm_dc_pc))-1)*dt,dm_dc_pc*100,'.k')
legend('Entropy','Heuristic')
xlabel('Time (s)')
ylabel('Improvement over baseline (%)')
figset
