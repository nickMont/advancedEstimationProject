function J = J_eva(x_eva,x_pur,u_eva,u_pur,C)

[~,k]=size(x_eva);
k=k-1;  %x_pur has size (nX, planningThreshold+1)
J=0;
for ik=1:k
    e=(x_eva(:,ij)-x_pur(:,ij));
    J = J + e'*C.Q*e + u_pur(:,ik)'*C.Rself*u_pur(:,ik) + ...
        u_eva(:,ik)*C.Ropp*u_eva(:,ik);
end

end
