function J = J_pur2p(x_pur,x_eva,u_pur,u_eva,C)

[~,k]=size(x_pur);
k=k-1;  %x_pur has size (nX, planningThreshold+1)
J=0;
[nx,~]=size(x_pur);
n=nx/2;
for ik=1:k
    e=(x_pur(1:n,ik+1)-x_eva(:,ik+1));
    J = J + e'*C.Q*e;
    e=(x_pur(n+1:end,ik+1)-x_eva(:,ik+1));
    J = J + e'*C.Q*e + u_pur(:,ik)'*C.Rself*u_pur(:,ik) + ...
        u_eva(:,ik)'*C.Ropp*u_eva(:,ik);
end

end

