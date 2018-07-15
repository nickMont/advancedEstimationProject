function [Jpur,Jeva,uOutPur,uOutEva] = generateCostMatricesQuads(Spur, Seva, gameState,vk)
%inputs:
%     structures Spur,Seva containing:
%        uMat   %Possible control value structure. uMat{ik} contains the
%                nUxK matrix of controls of the ik^th action to be applied
%                at time instant K
%        Jname  %string function name for cost function
%        fname  %string dynamics function name
%        Jparams  %struct containing parameters for J    
%        discType  %either 'overX' or 'overU' to determine whether the
%                   primary discretization in uMat is over control U or over output
%                   space X
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

numberOfSatSteps=2; %maximum number of tries to generate feasible control

Jpur=9001*ones(nmodP,nmodE);
Jeva=42*ones(nmodP,nmodE);

nx = length(gameState.xPur);
nv=nx/2;
state = zeros(nx,gameState.kMax+1);
xPurCell = cell(nmodP);
omegaRPur = cell(nmodP);

global errorFlag
errorFlag=false;

validSetP=[];
validSetE=[];
for ij=1:nmodP
    uLoc=[];
    state(:,1)=gameState.xPur;
    if strcmp(gameState.discType,'overU')
        validSetP=1:1:nmodP;
        for ik=1:gameState.kMax
            state(:,ik+1) = feval(Spur.fname,state(:,ik),Spur.uMat{ij}(:,ik),gameState.dt,zeros(nv,1));
            uLoc = [uLoc Spur.uMat{ij}(:,ik)];
        end
    elseif strcmp(gameState.discType,'overX')
        for ik=1:gameState.kMax
            dx = Spur.uMat{ij}(:,ik);
            des=state(:,ik);
            des(7:9)=des(7:9)+dx;
            [uu, validNum] = quadController(state(:,ik),zeros(4,1),zeros(3,1),des,1,zeros(12,1));
            uLoc = [uLoc uu];
            if validNum<=numberOfSatSteps
                validSetP=[validSetP ij];
            end
            state(:,ik+1) = feval(Spur.fname,state(:,ik),uu,gameState.dt,zeros(nv,1));
        end
    else
        fprintf('Unrecognized discretization type');
    end
    omegaRPur{ij} = uLoc;
    xPurCell{ij} = state;
end

nx = length(gameState.xEva);
state = zeros(nx,gameState.kMax+1);
xEvaCell = cell(nmodE);
omegaREva = cell(nmodE);
for ij=1:nmodE
    state(:,1)=gameState.xEva;
    uLoc = [];
    if strcmp(gameState.discType,'overU')
        validSetE=1:1:nmodE;
        for ik=1:gameState.kMax
            state(:,ik+1) = feval(Seva.fname,state(:,ik),Seva.uMat{ij}(:,ik),gameState.dt,zeros(nv,1));
            uLoc = [uLoc Seva.uMat{ij}(:,ik)];
        end
    elseif strcmp(gameState.discType,'overX')
        for ik=1:gameState.kMax
            dx = Seva.uMat{ij}(:,ik);
            des=state(:,ik);
            des(7:9)=des(7:9)+dx;
            [uu,validNum] = quadController(state(:,ik),zeros(4,1),zeros(3,1),des,1,zeros(12,1));
            uLoc = [uLoc uu];
            if validNum<=numberOfSatSteps
                validSetE=[validSetE ij];
            end
            state(:,ik+1) = feval(Spur.fname,state(:,ik),uu,gameState.dt,zeros(nv,1));
        end
    else
        fprintf('Unrecognized discretization type');
    end
    omegaREva{ij} = uLoc;
    xEvaCell{ij} = state;
end

noiseCostPur=3409;
noiseCostEva=3409;
nmodPvalid=length(validSetP);
nmodEvalid=length(validSetE);
Jpur=zeros(nmodPvalid,nmodEvalid);
Jeva=zeros(nmodPvalid,nmodEvalid);
for iPv=1:nmodPvalid
    for iEv=1:nmodEvalid
        iP=validSetP(iPv);
        iE=validSetE(iEv);
        fprintf('generating P/E: %d/%d \n',iP,iE);
        Jpur(iP,iE)=feval(Spur.Jname,xPurCell{iP},xEvaCell{iE},omegaRPur{iP},omegaREva{iE},Spur.Jparams);
        Jeva(iP,iE)=feval(Seva.Jname,xEvaCell{iE},xPurCell{iP},omegaREva{iE},omegaRPur{iP},Seva.Jparams);
    end
end

fprintf('%d and %d valid P/E controls\n',nmodPvalid,nmodEvalid);

uOutPur={};
uOutEva={};

if (strcmp(gameState.discType,'overX'))
    for iPv=1:nmodPvalid
        uOutPur{iPv}=omegaRPur{validSetP(iPv)};
    end
    for iEv=1:nmodEvalid
        uOutEva{iEv}=omegaREva{validSetE(iEv)};
    end
end

end

