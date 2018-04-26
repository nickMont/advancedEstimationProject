function [Jpur,Jeva] = generateCostMatrices(Spur, Seva, gameState,vk)
%inputs:
%     structures Spur,Seva containing:
%        uMat   %Possible control value structure. uMat{ik} contains the
%                nUxK matrix of controls of the ik^th action to be applied
%                at time instant K
%        Jname  %string function name for cost function
%        fname  %string dynamics function name
%        Jparams  %struct containing parameters for J    
%        
%
%     structure gameState containing:
%        xPur, xEva  %states of pursuer and evader
%        dt  %dt between steps
%        kMax  %number of time steps to plan over
%
%
%   Dynamics function f_dyn is referenced.  It can be changed if dynamics
%       are also changed. Likewise for Jpur(xpur,xeva,upur,ueva)

nmodP = length(Spur.uMat);
nmodE = length(Seva.uMat);

Jpur=9001*ones(nmodP,nmodE);
Jeva=42*ones(nmodP,nmodE);

nx = length(gameState.xPur);
nv=nx/2;
state = zeros(nx,gameState.kMax+1);
xPurCell = cell(nmodP);
for ij=1:nmodP
    state(:,1)=gameState.xPur;
    for ik=1:gameState.kMax
        state(:,ik+1) = feval(Spur.fname,state(:,ik),Spur.uMat{ij}(:,ik),gameState.dt,zeros(nv,1));
    end
    xPurCell{ij} = state;
end

nx = length(gameState.xEva);
state = zeros(nx,gameState.kMax+1);
xEvaCell = cell(nmodE);
for ij=1:nmodE
    state(:,1)=gameState.xEva;
    for ik=1:gameState.kMax
        state(:,ik+1) = feval(Seva.fname,state(:,ik),Seva.uMat{ij}(:,ik),gameState.dt,zeros(nv,1));
    end
    xEvaCell{ij} = state;
end

noiseCostPur=3409;
noiseCostEva=3409;
Jpur=zeros(nmodP,nmodE);
Jeva=zeros(nmodP,nmodE);
for iP=1:nmodP
    for iE=1:nmodE
        Jpur(iP,iE)=feval(Spur.Jname,xPurCell{iP},xEvaCell{iE},Spur.uMat{iP},Seva.uMat{iE},Spur.Jparams);
        Jeva(iP,iE)=feval(Seva.Jname,xEvaCell{iE},xPurCell{iP},Seva.uMat{iE},Spur.uMat{iP},Seva.Jparams);
    end
end



end

