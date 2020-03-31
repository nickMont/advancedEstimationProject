function [uPurIndex,uEvaIndex,outputflag,uPmat,uEmat,misc] = f_dynNN(Spur,Seva,gameState,vk,network1,network2)
if nargin<=3
    vk=zeros(100,1);
end
lenv=floor(length(vk)/2);
misc=[];

%Generate cost matrices
tic
[Cpur,Ceva,uPmat,uEmat]=generateCostMatricesNNv2(Spur,Seva,gameState,network1,network2);
t1=toc;

% VV=strictDominance(-Cpur,'row');
% sd_size_pur=size(VV)
% VV=strictDominance(-Ceva,'col');
% sd_size_eva=size(VV)
% 
% [VpTmp,VeTmp]=iterativeStrictDominance(-Cpur,-Ceva,100,-CpurMax,-CevaMax);
% [a,~]=size(Cpur)
% [a2,~]=size(VpTmp)
% [~,b]=size(Ceva)
% [~,b2]=size(VeTmp)
% dP=a-a2
% dE=b-b2

tic
%heuristics go here
[CpurUpper,CevaUpper]=generateCostMatricesUpper(Spur,Seva,gameState);
[CpurLower,~]=generateCostMatricesLowerForPursuer(Spur,Seva,gameState);
[~,CevaLower]=generateCostMatricesLowerForEvader(Spur,Seva,gameState);
VpurUpper=-CpurUpper;
VevaUpper=-CevaUpper;
VpurLower=-CpurLower;
VevaLower=-CevaLower;
[Vp2,Ve2,rowPrune,colPrune]=iterativeStrictDominance(VpurLower,VevaLower,100,VpurUpper,VevaUpper);
[sp1,se1]=size(Cpur);
[sp2,se2]=size(Vp2);
Spur_prune=Spur;
Seva_prune=Seva;
derp=generateCostMatricesNNv2(Spur_prune,Seva_prune,gameState,network1,network2);
timeHeuristic=toc;

misc=[t1;timeHeuristic;sp1*se1;sp2*se2];

%aa=LH2(-Cpur,-Ceva)
%lhPur=aa{1}
%lhEva=aa{2}
outputflag=1;
%Solve Nash
[rdeq,flag]=findRDEq(-Cpur,-Ceva);
sol2=LH2(-Cpur,-Ceva);
%R=Seva.Jparams.Rself
%mp=max(sol2{1})
%me=max(sol2{2})
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
    uPurIndex=sol{1};
    uEvaIndex=sol{2};
    outputflag=0;
else %unique solution found
    up_index=rdeq(1,1);
    ue_index=rdeq(2,1);
    uPurIndex=up_index;
    uEvaIndex=ue_index;
end

%xkp1_pur = f_dynPur(gameState.xPur,uPur,gameState.dt,vk(1:lenv));
%xkp1_eva = f_dynEva(gameState.xEva,uEva,gameState.dt,vk(lenv+1:lenv*2));
%xkp1_q = diag(Seva.Jparams.Q);
%xkp1_r = diag(Seva.Jparams.Rself);
%xkp1=[xkp1_pur; xkp1_eva; xkp1_q; xkp1_r];

end

