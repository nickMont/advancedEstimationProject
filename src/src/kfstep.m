function [xkp1,Pkp1] = kfstep(x,z,A,B,u,G,Q,P,H,R)
xbar=A*x+B*u;
Pbar=A*P*A'+G*Q*G';

zhat=H*xbar;
nu=z-zhat;
Sk=H*Pbar*H'+R;
Wk=Pbar*H'*inv(Sk);

xkp1=xbar+Wk*nu;
Pkp1=(eye(length(x))-Wk*H)*Pbar;

end

