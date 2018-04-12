function J = J_pur(x_pur,x_eva,u_pur,u_eva,C)

[~,k]=size(x_pur);
k=k-1;  %x_pur has size (nX, planningThreshold+1)
J=0;
for ik=1:k
    e=(x_pur(:,ij)-x_eva(:,ij));
    J = J + e'*C.Q*e + u_pur(:,ik)'*C.Rself*u_pur(:,ik) + ...
        u_eva(:,ik)*C.Ropp*u_eva(:,ik);
end

end

