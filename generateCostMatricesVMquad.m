function [Jpur,Jeva,uPcell,uEcell] = generateCostMatricesVMquad(Spur, Seva, gameState,vk)
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

if ~isfield(Spur,'UseVelMatch') || ~isfield(Seva,'UseVelMatch')
    error('HARDCODED TO ONLY HAND VM/GT HYBRID--Have you added a UseVelMatch flag to Spur/Seva?')
elseif ~Spur.UseVelMatch || ~Seva.UseVelMatch
    error('UseVelMatch set to false but generateCostMatricesVM2 was called')
end

nmodP = length(Spur.uMat);
nmodE = length(Seva.uMat);

Jpur=9001*ones(nmodP,nmodE);
Jeva=42*ones(nmodP,nmodE);

nx = length(gameState.xPur);
% state = zeros(nx,gameState.kMax+1);
xPurCell = cell(nmodP,nmodE);
xEvaCell = cell(nmodP,nmodE);
uPcell = cell(nmodP,nmodE);
uEcell = cell(nmodP,nmodE);
for ij=1:nmodP
    for iL=1:nmodE
        stateP(:,1)=gameState.xPur;
        stateE(:,1)=gameState.xEva;
        uPm=[];
        uEm=[];
        for ik=1:gameState.kMax
            xp=stateP([7 8 10 11],ik);
            xe=stateE([7 8 10 11],ik);
            if strcmp(Spur.controlType,'vmquad')
                [uhatVelMatch,states] = vmRGVO_tune(xp,xe,gameState.uMaxP,2,gameState.dt,zeros(2,1),1);
                uhatVelMatch = unit_vector(uhatVelMatch);
                %limit vector to quadrants 1+4
                theta=atan2(uhatVelMatch(2),uhatVelMatch(1));
                if theta>pi/2 && theta<3*pi/2
                    uhatVelMatch=-uhatVelMatch;
                end
                xPt=states.xPout;
                uPtilde=uhatVelMatch*norm(Spur.uMat{ij}(:,ik));
                xd = stateP(:,ik); xd(9)=0;
                uP = quadControllerACCONLY(xd, zeros(4,1), 3, [uPtilde;0],0);
            elseif strcmp(Spur.controlType,'gt_overx')
                dx = Spur.uMat{ij}(:,ik);
                des = stateP(:,ik);
                des(7:8) = des(7:8)+dx;
                [uP, validNum] = quadController(stateP(:,ik),zeros(4,1),zeros(3,1),des,1,zeros(12,1)); %#ok
            else
                error('Failed to present a valid gameState.controlType');
            end
            
            if strcmp(Seva.controlType,'vmquad')
                [uhatVelMatch,states] = vmRGVO_tune(xp,xe,gameState.uMaxP,2,gameState.dt,zeros(2,1),1);
                uhatVelMatch = unit_vector(uhatVelMatch);
                %limit vector to quadrants 1+4
                theta=atan2(uhatVelMatch(2),uhatVelMatch(1));
                if theta>pi/2 && theta<3*pi/2
                    uhatVelMatch=-uhatVelMatch;
                end
                xEt=states.xEout;
                uEtilde=uhatVelMatch*norm(Seva.uMat{iL}(:,ik));
                xd = stateE(:,ik); xd(9)=0;
                uE = quadControllerACCONLY(xd, zeros(4,1), 3, [uEtilde;0],0);
            elseif strcmp(Seva.controlType,'gt_overx')
                dx = Seva.uMat{iL}(:,ik);
                des = stateE(:,ik);
                des(7:8) = des(7:8)+dx;
                [uE, validNum] = quadController(stateE(:,ik),zeros(4,1),zeros(3,1),des,1,zeros(12,1)); %#ok
            else
                error('Failed to present a valid gameState.controlType');
            end
            
            %P
            stateP(:,ik+1) = feval(Spur.fname,stateP(:,ik),uP,gameState.dt,zeros(6,1));
            stateE(:,ik+1) = feval(Spur.fname,stateE(:,ik),uE,gameState.dt,zeros(6,1));
            uPm=[uPm uP];
            uEm=[uEm uE];
        end
        uPcell{ij,iL} = uPm;
        uEcell{ij,iL} = uEm;
        xPurCell{ij,iL} = stateP;
        xEvaCell{ij,iL} = stateE;
    end
end

% nx = length(gameState.xEva);
% state1 = zeros(nx,gameState.kMax+1);
% 
% for ij=1:nmodE
%     state(:,1)=gameState.xEva;
%     for ik=1:gameState.kMax
%         if isfield(Seva,'params')
%             state(:,ik+1) = feval(Seva.fname,state(:,ik),Seva.uMat{ij}(:,ik),gameState.dt,zeros(nv,1),Seva.params);
%         else
%             state(:,ik+1) = feval(Seva.fname,state(:,ik),Seva.uMat{ij}(:,ik),gameState.dt,zeros(nv,1));
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
        Jpur(iP,iE)=feval(Spur.Jname,xPurCell{iP,iE},xEvaCell{iP,iE},uPcell{iP,iE},uEcell{iP,iE},Spur.Jparams);
        Jeva(iP,iE)=feval(Seva.Jname,xEvaCell{iP,iE},xPurCell{iP,iE},uEcell{iP,iE},uPcell{iP,iE},Seva.Jparams);
    end
end


end

