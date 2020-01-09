function [Jpur,Jeva,uMatCellP,uMatCellE] = generateCostMatricesNNv2(Spur, Seva, gameState, network1, network2)
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

[xPurCell, uMatCellP] = generateStateMatFromPairings(pairingsP,Spur,Seva,gameState,network1,network2);
[xEvaCell, uMatCellE] = generateStateMatFromPairings(pairingsE,Spur,Seva,gameState,network1,network2);
% for ij=1:nmodP
%     nP=size(pairingsP{ij});
%     for iL=1:nP
%         this_numP=length(pairingsP{ij}{1});
%     
%         %set up initial state
%         ministate=[];
%         for iM=1:size(pairingsP{iL}(1))
%             purIndex = pairingsP{iL}{1}(iM);
%             ministate=[ministate; gameState.xPur{purIndex}];
%             state(:,purIndex,1)=gameState.xPur{purIndex};
%         end
%         for iM=1:size(pairingsP{iL}(2))
%             evaIndex=pairingsP{iL}{2}(iM);
%             ministate=[ministate; gameState.xPur{evaIndex}];
%             state(:,numP+evaIndex,1)=gameState.xEva{evaIndex};
%         end
%         Qp=Spur.lowlevel.Q;
%         Qe=Seva.lowlevel.Q;
%         Rp=Spur.lowlevel.R;
%         Re=Seva.lowlevel.R;
%         
%         if this_numP==1
%             qqrr=0.1*[diag(Qp) diag(Qe) diag(Rp) diag(Re)]'; %0.1 was tuning param in trainPE
%         else
%             qqrr=0.1*[diag(Qp) diag(Qp) diag(Qe) diag(Rp) diag(Rp) diag(Re)]';
%         end          
% 
%         %propagate this pairing
%         for ik=1:gameState.kMax
%             %propate for 1v1
%             if this_numP==1
%                 uVec=predict(network1,[ministate;qqrr]);
%                 ministate(1:nx) = feval(Spur.fname{purIndex},ministate(1:nx),uVec(1:nu),gameState.dt,zeros(nv,1));
%                 ministate(nx+1:end) = feval(Seva.fname{evaIndex},ministate(nx+1:end),uVec(nu+1:end),gameState.dt,zeros(nv,1));
%             else %propagate for Xv1
%                 uVec=predict(network2,[ministate;qqrr]);
%                 for iM=1:2
%                     purIndex = pairingsP{iL}{1}(iM);
%                     ministate((iM-1)*nx+1:iM*nx) = feval(Spur.fname{purIndex},ministate((iM-1)*nx+1:iM*nx),uVec(((iM-1)*nu+1:iM*nu)),gameState.dt,zeros(nv,1));
%                 end
%                 ministate(2*nx+1:end) = feval(Spur.fname{evaIndex},ministate(2*nx+1:end),uVec(2*nu+1:end),gameState.dt,zeros(nv,1));
%             end
%             
%             %pack into state
%             for iM=1:this_numP
%                 state(:,pairingsP{iL}{1}(iM),1)=ministate((iM-1)*nx+1:iM*nx);
%             end
%             state(:,numP+pairingsE{iL}{2}(1),1)=ministate(this_numpP*nx+1:end);
%         end
%     end
%     xPurCell{ij} = state;
% end
% 
% nx = length(gameState.xEva);
% state = zeros(nx,numP+numE,gameState.kMax+1);
% xEvaCell = cell(nmodE);
% for ij=1:nmodE
%     nP=size(Spur.pairings.P);
%     nE=size(Spur.pairings.E);
%     for iL=1:length(Seva.pairings.P)
%         ministate=[];
%         for iM=1:size(Seva.pairings.P{iL})
%             ministate=[ministate; gameState.xPur{Seva.pairings.P{iL}{iM}}];
%             state(:,Spur.pairings.P{iL}{iM},1)=gameState.xPur{Seva.pairings.P{iL}{iM}};
%         end
%         for iM=1:size(Seva.pairings.E{iL})
%             ministate=[ministate; gameState.xPur{Seva.pairings.E{iL}{iM}}];
%             state(:,numP+Spur.pairings.E{iL}{iM},1)=gameState.xEva{Seva.pairings.E{iL}{iM}};
%         end
%         Qp=Spur.lowlevel.Q;
%         Qe=Seva.lowlevel.Q;
%         Rp=Spur.lowlevel.R;
%         Re=Seva.lowlevel.R;
%         if length(Seva.pairings.P{iL}{iM})==1
%             qqrr=0.1*[diag(Qp) diag(Qe) diag(Rp) diag(Re)]'; %0.1 was tuning param in trainPE
%         else
%             qqrr=0.1*[diag(Qp) diag(Qp) diag(Qe) diag(Rp) diag(Rp) diag(Re)]';
%         end
%         %    uP=uVec(1:2);uE=uVec(3:4);
%         for ik=1:gameState.kMax
%             if length(Seva.pairings.P{iL}{iM})==1
%                 uVec=predict(network1,[state(:,ik);qqrr]);
%             else
%                 uVec=predict(network2,[state(:,ik);qqrr]);
%             end
%             ministate = feval(Seva.fname,ministate,uVec,gameState.dt,zeros(nv,1),length(Seva.pairings.P{iL}{iM}));
%             for iM=1:nP
%                 state(:,Seva.pairings.P{iL}{iM},1)=ministate((iM-1)*nx+1:iM*nx);
%             end
%             for iM=1:nE
%                 state(:,numP+Seva.pairings.E{iL}{iM},1)=ministate(nP*nx+(iM-1)*nx+1:nP*nx+iM*nx);
%             end
%         end
%     end
%     xEvaCell{ij} = state;
% end

noiseCostPur=3409;
noiseCostEva=3409;
Jpur=zeros(nmodP,nmodE);
Jeva=zeros(nmodP,nmodE);
for iP=1:nmodP
    for iE=1:nmodE
        Jpur(iP,iE)=feval(Spur.Jname,xPurCell{iP},xEvaCell{iE},Spur.uMat{iP},Seva.uMat{iE},Spur.Jparams,gameState);
        Jeva(iP,iE)=feval(Seva.Jname,xPurCell{iP},xEvaCell{iE},Seva.uMat{iE},Spur.uMat{iP},Seva.Jparams,gameState);
    end
end



end

