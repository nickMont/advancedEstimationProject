function [uPur,uEva,outputflag,uValP,uValE] = f_dyn2(Spur,Seva,gameState,vk)
if nargin<=3
    vk=zeros(100,1);
end
lenv=floor(length(vk)/2);

%Generate cost matrices
[Cpur,Ceva,uP,uE]=generateCostMatricesQuads(Spur,Seva,gameState);

%aa=LH2(-Cpur,-Ceva)
%lhPur=aa{1}
%lhEva=aa{2}
outputflag=1;
%Solve Nash
%[rdeq,flag]=findRDEq(-Cpur,-Ceva);

flag=0;

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

uValP=[];
uValE=[];
if ~uP.isempty
    uInd=randsample(length(sol{1}),1,true,sol{1});
    uValP=uP(:,uInd);
    uInd=randsample(length(sol{2}),1,true,sol{2});
    uValE=uE(:,uInd);    
end

end

