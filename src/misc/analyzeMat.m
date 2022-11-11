clear all;clc;close all

load test50_v2.mat

dJmat=[];
rmse2sum=0;
for ij=1:20
    
    xPur_part=xPartStore{ij};
    
    meann=zeros(14,1);
    for ik=1:npart
        meann=meann+wloc(ik)*xPur_part(:,ik);
    end
    %dx=xPur-meann(1:8)
    dJ=meann(9:14)-qrTrue;
    rmse2sum=rmse2sum+dJ.^2/21;
    dJmat=[dJmat abs(dJ)];
end
rmse2sum=sqrt(rmse2sum);

for ik=1:3
    figure(1);
    subplot(3,1,ik)
    hold on
    plot(1:1:20,dJmat(2*(ik-1)+1,:),'r')
    hold on
    plot(1:1:20,dJmat(2*(ik-1)+2,:),'b')
    if ik==1
        legend('eQ_{xx}','eQ_{yy}')
    elseif ik==2
        legend('eQ_{vx}','eQ_{vy}')
    else
        legend('eR_{xx}','eR_{yy}')
    end
end

xT=[];
for ij=1:21
    xT=[xT xTrueS{ij}];
end
figure(2);clf;
plot(xT(1,:),xT(2,:),'r--o')
hold on
plot(xT(5,:),xT(6,:),'b--o')
legend('x_{pr}','x_{ev}')


