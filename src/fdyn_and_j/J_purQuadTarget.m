function J = J_purQuadTarget(x_pur,x_eva,u_pur,u_eva,C)

%[~,k]=size(x_pur);
%k=k-1;  %x_pur has size (nX, planningThreshold+1)
J=0;
k=1;
for ik=1:k
    e=(x_pur(:,ik+1)-x_eva(:,ik+1));
    J = J + e'*C.Q*e + u_pur(:,ik)'*C.Rself*u_pur(:,ik) + ...
        u_eva(:,ik)'*C.Ropp*u_eva(:,ik);
    if( norm(x_pur-9001*ones(12,1)) < 1e-5)
        J=J+9001;
    end
    if( norm(x_eva-9001*ones(12,1)) < 1e-5)
        J=J+9001;
    end
end
J=J-(x_eva(7:9,k+1)-C.x_target)'*C.Q_target*(x_eva(7:9,k+1)-C.x_target);

end

