function [uPur,uEva,outputflag] = f_dyn(Spur,Seva,gameState,vk)
if nargin<=3
    vk=zeros(100,1);
end
lenv=floor(length(vk)/2);

%Generate cost matrices
[Cpur,Ceva]=generateCostMatrices(Spur,Seva,gameState);

%aa=LH2(-Cpur,-Ceva)
%lhPur=aa{1}
%lhEva=aa{2}
outputflag=1;
%Solve Nash
[rdeq,flag]=findRDEq(-Cpur,-Ceva);
if flag==0 %if no unique solution, run LH2 and take E(u) for result
    sol=LH2(-Cpur,-Ceva);
%     uPur = zeros(gameState.nu,1);
%     uEva = zeros(gameState.nu,1);
%     for ij=1:length(sol{1})
%         uPur=uPur + sol{1}(ij)*Spur.uMat{ij};
%     end
%     for ij=1:length(sol{2})
%         uEva = uEva + sol{2}(ij)*Seva.uMat{ij};
%     end
    uPur=sol{1};
    uEva=sol{2};
    outputflag=0;
else %unique solution found
    up_index=rdeq(1,1);
    ue_index=rdeq(2,1);
    uPur=Spur.uMat{up_index};
    uEva=Seva.uMat{ue_index};
end

%xkp1_pur = f_dynPur(gameState.xPur,uPur,gameState.dt,vk(1:lenv));
%xkp1_eva = f_dynEva(gameState.xEva,uEva,gameState.dt,vk(lenv+1:lenv*2));
%xkp1_q = diag(Seva.Jparams.Q);
%xkp1_r = diag(Seva.Jparams.Rself);
%xkp1=[xkp1_pur; xkp1_eva; xkp1_q; xkp1_r];

end

