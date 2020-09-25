function J = J_eva2p(x_eva,x_pur,u_eva,u_pur,C)

[~,k]=size(x_eva);
k=k-1;  %x_pur has size (nX, planningThreshold+1)
J=0;
[nx,~]=size(x_pur);
n=nx/2;
for ik=1:k
    e=(x_pur(1:n,ik+1)-x_eva(:,ik+1));
    J = J + e'*C.Q*e;
    e=(x_pur(n+1:end,ik+1)-x_eva(:,ik+1));
    J = J + -e'*C.Q*e + u_eva(:,ik)'*C.Rself*u_eva(:,ik) + ...
        u_pur(:,ik)'*C.Ropp*u_pur(:,ik);
end

end
