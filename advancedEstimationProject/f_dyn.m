function [uPur,uEva,outputflag,tmp1,tmp2,miscOutputs] = f_dyn(Spur,Seva,gameState,vk)
if nargin<=3
    vk=zeros(100,1);
end
lenv=floor(length(vk)/2);

miscOutputs=[];


% tic
% %Generate cost matrices
% [Cpur_UB,Ceva_UB]=generateCostMatricesUpper(Spur,Seva,gameState);
% [Cpur_LB,~]=generateCostMatricesLowerForPursuer(Spur,Seva,gameState);
% [~,Ceva_LB]=generateCostMatricesLowerForEvader(Spur,Seva,gameState);
% [~,~,a,b]=iterativeStrictDominance(-Cpur_UB,-Ceva_UB,100,-Cpur_LB,-Ceva_LB);
% SpurTilde=Spur;
% SevaTilde=Seva;
% SpurTilde.uMat=SpurTilde.uMat{a};
% SevaTilde.uMat=SevaTilde.uMat{b};
% [CpurT,CevaT]=generateCostMatrices(SpurTilde,SevaTilde,gameState);
% %Solve Nash
% [rdeq,flag]=findRDEq(-CpurT,-CevaT);
% sol2=LH2(-CpurT,-CevaT);
% timeHeuristic=toc;

% tic
% Generate cost matrices
%
if ~isfield(Spur,'UseVelMatch') || ~isfield(Seva,'UseVelMatch') || ~Spur.UseVelMatch || ~Seva.UseVelMatch
    [Cpur,Ceva]=generateCostMatrices(Spur,Seva,gameState);
else
    [Cpur,Ceva,uP,uE]=generateCostMatricesVM2(Spur,Seva,gameState);
    miscOutputs.uP=uP;
    miscOutputs.uE=uE;
end
% aa=LH2(-Cpur,-Ceva)
% lhPur=aa{1}
% lhEva=aa{2}
outputflag=1;
% Solve Nash
[rdeq,flag]=findRDEq(-Cpur,-Ceva);
sol2=LH2(-Cpur,-Ceva);
% R=Seva.Jparams.Rself
% mp=max(sol2{1})
% me=max(sol2{2})
% timeDefault=toc;
% miscOutputs=[timeHeuristic; timeDefault];
sol=sol2;

tmp1=[];
tmp2=[];
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
    tmp1=up_index;
    tmp2=ue_index;
end

%xkp1_pur = f_dynPur(gameState.xPur,uPur,gameState.dt,vk(1:lenv));
%xkp1_eva = f_dynEva(gameState.xEva,uEva,gameState.dt,vk(lenv+1:lenv*2));
%xkp1_q = diag(Seva.Jparams.Q);
%xkp1_r = diag(Seva.Jparams.Rself);
%xkp1=[xkp1_pur; xkp1_eva; xkp1_q; xkp1_r];

end

