function [state0,augmented_state_struct] = filterCostFunctions(augmented_state_struct, zk, uPur,uEva, purGameMatrices)

num_of_distributions=size(augmented_state_struct);
fname_eva = "f_dynEva.m";
hname = "hmeas.m";
Spur.fname = "f_dynPur.m";
Spur.Jname = "J_pur.m";
Spur.uMat=uPur;
Spur.Jparams=purGameMatrices;
Seva.fname = "f_dynEva.m";
Seva.Jname = "J_eva.m";
Seva.uMat=uEva;
for ij=1:num_of_distributions
    
    [state,Pk]=runUKF(augmented_state_struct{ij}.state,...
        augmented_state_struct{ij}.Pk,zk, Rk,Qk,Spur,Seva,dt,1);
%Augmented state: [xPur; xEva; qmatEva; rmatEva];    

end


end

