function [xhat,Phat,nu,Sk,Wk] = kfMeasure(xbar,Pbar,zk,Hk,Rk)

nu = zk-Hk*xbar;
Sk = Hk*Pbar*Hk'+Rk;
Wk = Pbar*Hk'*inv(Sk);
xhat = xbar+Wk*nu;
Phat = (eye(length(xbar))-Wk*Hk)*Pbar;

end

