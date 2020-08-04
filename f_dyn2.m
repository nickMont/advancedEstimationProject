function [uPur,uEva,outputflag,uValP,uValE,Sminimax] = f_dyn2(Spur,Seva,gameState,vk)
if nargin<=3
    vk=zeros(100,1);
end
lenv=floor(length(vk)/2);
Sminimax=[];

%Generate cost matrices
[Cpur,Ceva,uP,uE]=generateCostMatricesVMquad(Spur,Seva,gameState);
[nP,nE]=size(Cpur);

[indminimax,~,~]=minimax2(Cpur,Ceva);
Sminimax.index=indminimax;
Sminimax.uP=uP{indminimax(1),indminimax(2)};
Sminimax.uE=uE{indminimax(1),indminimax(2)};

%aa=LH2(-Cpur,-Ceva)
%lhPur=aa{1}
%lhEva=aa{2}
%Solve Nash
[rdeq,flag]=findRDEq(-Cpur,-Ceva);
outputflag=flag;

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
    uValP=[];
    uValE=[];
    if ~(isempty(uP))
        uInd=randsample(nP,1,true,uPur);
        uValP=uP{uInd};
        uInd=randsample(nE,1,true,uEva);
        uValE=uE{uInd};
    end
else %unique solution found
    up_index=rdeq(1,1);
    ue_index=rdeq(2,1);
    uPur=Spur.uMat{up_index};
    uEva=Seva.uMat{ue_index};
    uValP=uP{up_index,ue_index};
    uValE=uE{up_index,ue_index};
end

end

