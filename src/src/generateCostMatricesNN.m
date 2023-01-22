function [Jpur,Jeva] = generateCostMatricesNN(Spur, Seva, gameState, network1, network2)
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

%trained_net_vars = load('peNN.mat');
%handles.neural_net = trained_net_vars.net;
%uDes = predict(handles.neural_net,X);

nmodP = length(Spur.uMat);
nmodE = length(Seva.uMat);

Jpur=9001*ones(nmodP,nmodE);
Jeva=42*ones(nmodP,nmodE);

nx = length(gameState.xPur);
nv = nx/2;
nu = nv;
numP = gameState.numPursuers;
numE = gameState.numEvaders;
state = zeros(nx,numP+numE,gameState.kMax+1);
uMat = zeros(nu,numP+numE,gameState.kMax+1);
xPurCell = cell(nmodP);
for ij=1:nmodP
    nP=size(Spur.pairings.P);
    nE=size(Spur.pairings.E);
    for iL=1:length(Spur.pairings.P{iL})
        ministate=[];
        for iM=1:size(Spur.pairings.P{iL})
            ministate=[ministate; gameState.xPur{Spur.pairings.P{iL}{iM}}];
            state(:,Spur.pairings.P{iL}{iM},1)=gameState.xPur{Spur.pairings.P{iL}{iM}};
        end
        for iM=1:size(Spur.pairings.E{iL})
            ministate=[ministate; gameState.xPur{Spur.pairings.E{iL}{iM}}];
            state(:,numP+Spur.pairings.E{iL}{iM},1)=gameState.xEva{Spur.pairings.E{iL}{iM}};
        end
        Qp=Spur.lowlevel.Q;
        Qe=Seva.lowlevel.Q;
        Rp=Spur.lowlevel.R;
        Re=Seva.lowlevel.R;
        if length(Spur.pairings.P{iL}{iM})==1
            qqrr=0.1*[diag(Qp) diag(Qe) diag(Rp) diag(Re)]'; %0.1 was tuning param in trainPE
        else
            qqrr=0.1*[diag(Qp) diag(Qp) diag(Qe) diag(Rp) diag(Rp) diag(Re)]';
        end
        %    uP=uVec(1:2);uE=uVec(3:4);
        for ik=1:gameState.kMax
            if length(Spur.pairings.P{iL}{iM})==1
                uVec=predict(network1,[state(:,ik);qqrr]);
            else
                uVec=predict(network2,[state(:,ik);qqrr]);
            end
            ministate = feval(Spur.fname,ministate,uVec,gameState.dt,zeros(nv,1),length(Spur.pairings.P{iL}{iM}));
            for iM=1:nP
                state(:,Spur.pairings.P{iL}{iM},1)=ministate((iM-1)*nx+1:iM*nx);
            end
            for iM=1:nE
                state(:,numP+Spur.pairings.E{iL}{iM},1)=ministate(nP*nx+(iM-1)*nx+1:nP*nx+iM*nx);
            end
        end
    end
    xPurCell{ij} = state;
end

nx = length(gameState.xEva);
state = zeros(nx,numP+numE,gameState.kMax+1);
xEvaCell = cell(nmodE);
for ij=1:nmodE
    nP=size(Spur.pairings.P);
    nE=size(Spur.pairings.E);
    for iL=1:length(Seva.pairings.P)
        ministate=[];
        for iM=1:size(Seva.pairings.P{iL})
            ministate=[ministate; gameState.xPur{Seva.pairings.P{iL}{iM}}];
            state(:,Spur.pairings.P{iL}{iM},1)=gameState.xPur{Seva.pairings.P{iL}{iM}};
        end
        for iM=1:size(Seva.pairings.E{iL})
            ministate=[ministate; gameState.xPur{Seva.pairings.E{iL}{iM}}];
            state(:,numP+Spur.pairings.E{iL}{iM},1)=gameState.xEva{Seva.pairings.E{iL}{iM}};
        end
        Qp=Spur.lowlevel.Q;
        Qe=Seva.lowlevel.Q;
        Rp=Spur.lowlevel.R;
        Re=Seva.lowlevel.R;
        if length(Seva.pairings.P{iL}{iM})==1
            qqrr=0.1*[diag(Qp) diag(Qe) diag(Rp) diag(Re)]'; %0.1 was tuning param in trainPE
        else
            qqrr=0.1*[diag(Qp) diag(Qp) diag(Qe) diag(Rp) diag(Rp) diag(Re)]';
        end
        %    uP=uVec(1:2);uE=uVec(3:4);
        for ik=1:gameState.kMax
            if length(Seva.pairings.P{iL}{iM})==1
                uVec=predict(network1,[state(:,ik);qqrr]);
            else
                uVec=predict(network2,[state(:,ik);qqrr]);
            end
            ministate = feval(Seva.fname,ministate,uVec,gameState.dt,zeros(nv,1),length(Seva.pairings.P{iL}{iM}));
            for iM=1:nP
                state(:,Seva.pairings.P{iL}{iM},1)=ministate((iM-1)*nx+1:iM*nx);
            end
            for iM=1:nE
                state(:,numP+Seva.pairings.E{iL}{iM},1)=ministate(nP*nx+(iM-1)*nx+1:nP*nx+iM*nx);
            end
        end
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

