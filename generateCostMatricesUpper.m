function [Jpur,Jeva,uMatCellP,uMatCellE] = generateCostMatricesUpper(Spur, Seva, gameState)
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
%
%   Spur and Seva both contain pairings structure.  pairings consists of N
%       cells.  Each cell contains two vectors.  pairings{i}{1} contains
%       the pursuer numbers assigned to the ith pairing.  pairings{i}{2}
%       contains the evader number assigned to the ith pairing.

%trained_net_vars = load('peNN.mat');
%handles.neural_net = trained_net_vars.net;
%uDes = predict(handles.neural_net,X);

pairingsP = Spur.pairings;
pairingsE = Seva.pairings;

nmodP = length(pairingsP);
nmodE = length(pairingsE);

Jpur=9001*ones(nmodP,nmodE);
Jeva=42*ones(nmodP,nmodE);

nx = length(gameState.xPur{1});
nv = nx/2;
nu = nv;
numP = gameState.numPursuers;
numE = gameState.numEvaders;
state = zeros(nx,numP+numE,gameState.kMax+1);
uMat = zeros(nu,numP+numE,gameState.kMax+1);
xPurCell = cell(nmodP);
xEvaCell = cell(nmodE);

[xPurCell, uMatCellP] = generateStateMatFromPairingsUpper(pairingsP,Spur,Seva,gameState);
[xEvaCell, uMatCellE] = generateStateMatFromPairingsUpper(pairingsE,Spur,Seva,gameState);

noiseCostPur=3409;
noiseCostEva=3409;
Jpur=zeros(nmodP,nmodE);
Jeva=zeros(nmodP,nmodE);
for iP=1:nmodP
    for iE=1:nmodE
        inactiveSetP=gameState.inactiveSetP(iP,:);
        inactiveSetP=inactiveSetP(inactiveSetP(:)~=0);
        inactiveSetE=gameState.inactiveSetE(iE,:);
        inactiveSetE=inactiveSetE(inactiveSetE(:)~=0);
        Jpur(iP,iE)=feval(Spur.Jname,xPurCell{iP},xEvaCell{iE},uMatCellP{iP},Spur.Jparams,gameState,inactiveSetP,inactiveSetE);
        Jeva(iP,iE)=feval(Seva.Jname,xPurCell{iP},xEvaCell{iE},uMatCellE{iE},Seva.Jparams,gameState,inactiveSetP,inactiveSetE);
    end
end



end

