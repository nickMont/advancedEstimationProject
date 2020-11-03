function J = J_evaQuad(x_eva,x_pur,u_eva,u_pur,C)

%[~,k]=size(x_eva);
%k=k-1;  %x_pur has size (nX, planningThreshold+1)
J=0;
k=1;
for ik=1:k
    e=(x_eva(:,ik+1)-x_pur(:,ik+1)); %look for NEXT location
    J = J + -e'*C.Q*e + u_eva(:,ik)'*C.Rself*u_eva(:,ik) + ...
        u_pur(:,ik)'*C.Ropp*u_pur(:,ik);
    if( norm(x_pur-9001*ones(12,1)) < 1e-5)
        J=J+9001;
    end
    if( norm(x_eva-9001*ones(12,1)) < 1e-5)
        J=J+9001;
    end    
end

end
