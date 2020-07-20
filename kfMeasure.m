function [xhat,Phat] = kfMeasure(xbar,Pbar,zk,Hk,Rk)

nu = zk-Hk*xbar;
Sk = Hk*Pbar*Hk'+Rk;
K = P*H'*inv(S);
xhat = xbar+K*nu;
Phat = (eye(length(xbar))-K*H)*Pbar;

end

