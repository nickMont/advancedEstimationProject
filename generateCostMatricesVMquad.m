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

uEvaEst=zeros(2,1);
if isfield(gameState,'uEvaEstForVM')
    uEvaEst=gameState.uEvaEstForVM;
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
if gameState.kMax>1
    statePcell=cell(nmodP,nmodE);
    stateEcell=cell(nmodP,nmodE);
end

%For all controllers, player dynamics is independent for k=1.  Process
% the pursuer and the evader independently, then merge the loop for k>1
% for efficiency.

Seva.Jparams

if isfield(gameState,'Rtarget')
    %load prediction params for P
    Rt_localP=gameState.Rtarget;
    Rt_localP.Qtarget = Seva.Jparams.Q_target(1:2,1:2); %2D
    Rt_localP.xTarget = gameState.Rtarget.x_target(1:2); %2D
    Rt_localP.Qpur=Spur.Jparams.Q(7:8,7:8);
    %for E
    Rt_localE=gameState.Rtarget;
    Rt_localE.Qtarget = Seva.Jparams.Q_target(1:2,1:2); %2D
    Rt_localE.xTarget = gameState.Rtarget.x_target(1:2); %2D
    Rt_localE.Qpur = Spur.Jparams.Q(7:8,7:8); %evader uses pursuer's Q for prediction
end

%For pursuer, dt=1
for ij=1:nmodP
    stateP(:,1)=gameState.xPur;
    stateE(:,1)=gameState.xEva;
    uPm=[];
    xp=stateP([7 8 10 11],1);
    xe=stateE([7 8 10 11],1);
    if strcmp(Spur.controlType,'vmquad')
        if isfield(gameState,'Rtarget')
            if gameState.Rtarget.useNoHeuristics && ~gameState.Rtarget.useMotionPrediction
                [uhatVelMatch,states] = vmRGVO_tune(xp,xe,gameState.uMaxP,2,gameState.dt,zeros(2,1),1);
            elseif gameState.Rtarget.useNoHeuristics && gameState.Rtarget.useMotionPrediction
                [uhatVelMatch,states] = vmRGVO_tune(xp,xe,gameState.uMaxP,2,gameState.dt,uEvaEst,1,Rt_localE);
                uhatVelMatch=unit_vector(uhatVelMatch);
            elseif ~gameState.Rtarget.useNoHeuristics && ~gameState.Rtarget.useMotionPrediction
                [uhatVelMatch,states] = vmRGVO_tune(xp,xe,gameState.uMaxP,2,gameState.dt,zeros(2,1),1);
                uhatVelMatch=unit_vector(uhatVelMatch); dxe=xp-xe; dxe=dxe(1:2);
                dxt = Rt_localE.xTarget-xe(1:2);
                uhatVelMatch=uhatVelMatch*(dxe'*Seva.Jparams.Q(7:8,7:8)*dxe)+unit_vector(dxt)*(dxt'*Rt_localE.Qpur*dxt);
            else
                [uhatVelMatch,states] = vmRGVO_tune(xp,xe,gameState.uMaxP,2,gameState.dt,uEvaEst,1,Rt_localE);
                uhatVelMatch=unit_vector(uhatVelMatch); dxe=xp-xe; dxe=dxe(1:2);
                dxt = Rt_localE.xTarget-xe(1:2);
                uhatVelMatch=uhatVelMatch*(dxe'*Seva.Jparams.Q(7:8,7:8)*dxe)+unit_vector(dxt)*(dxt'*Rt_localE.Qpur*dxt);
            end
        else
            [uhatVelMatch,states] = vmRGVO_tune(xp,xe,gameState.uMaxP,2,gameState.dt,uEvaEst,1);
        end
        %         [uhatVelMatch,states] = vmRGVO_tune(xp,xe,gameState.uMaxP,2,gameState.dt,uEvaEst,1);
        uhatVelMatch = unit_vector(uhatVelMatch);
        %limit vector to quadrants 1+4
        theta=atan2(uhatVelMatch(2),uhatVelMatch(1));
        if theta>pi/2 && theta<3*pi/2
            uhatVelMatch=-uhatVelMatch;
        end
        xPt=states.xPout;
        uPtilde=uhatVelMatch*norm(Spur.uMat{ij}(:,1));
        xd = stateP(:,1); xd(9)=0;
        uP = quadControllerACCONLY(xd, zeros(4,1), 3, [uPtilde;0],0);
    elseif strcmp(Spur.controlType,'gt_overx')
        dx = Spur.uMat{ij}(:,1);
        des = stateP(:,1);
        des(7:8) = des(7:8)+dx;
        [uP, validNum] = quadController(stateP(:,1),zeros(4,1),zeros(3,1),des,1,zeros(12,1)); %#ok
    elseif strcmp(Spur.controlType,'gt_overu')
        uP=Spur.uMat{ij}(:,1);
    else
        error('Failed to present a valid gameState.controlType');
    end
    
    %P
    if isfield(gameState,'tryNN')
        if gameState.tryNN
            stateP(:,2) = predict(gameState.NN,[stateP(:,1);uP]);
        else
            stateP(:,2) = feval(Spur.fname,stateP(:,1),uP,gameState.dt,zeros(6,1));
        end
    else
        stateP(:,2) = feval(Spur.fname,stateP(:,1),uP,gameState.dt,zeros(6,1));
    end
    uPm=[uPm uP];
    for iL=1:nmodE
        uPcell{ij,iL} = uPm;
        xPurCell{ij,iL} = stateP;
        if gameState.kMax>1
            statePcell{ij,iL}=stateP;
        end
    end
end

%For evader, dt=1
for iL=1:nmodE
    stateE(:,1)=gameState.xEva;
    uEm=[];
    xp=stateP([7 8 10 11],1);
    xe=stateE([7 8 10 11],1);
    
    if strcmp(Seva.controlType,'vmquad')
        if isfield(gameState,'Rtarget')
            if gameState.Rtarget.useNoHeuristics && ~gameState.Rtarget.useMotionPrediction
                [uhatVelMatch,states] = vmRGVO_tune(xp,xe,gameState.uMaxP,2,gameState.dt,zeros(2,1),1);
            elseif gameState.Rtarget.useNoHeuristics && gameState.Rtarget.useMotionPrediction
                [uhatVelMatch,states] = vmRGVO_tune(xp,xe,gameState.uMaxP,2,gameState.dt,uEvaEst,1,Rt_localE);
                uhatVelMatch=unit_vector(uhatVelMatch);
            elseif ~gameState.Rtarget.useNoHeuristics && ~gameState.Rtarget.useMotionPrediction
                [uhatVelMatch,states] = vmRGVO_tune(xp,xe,gameState.uMaxP,2,gameState.dt,zeros(2,1),1);
                uhatVelMatch=unit_vector(uhatVelMatch); dxe=xp-xe; dxe=dxe(1:2);
                dxt = Rt_localE.xTarget-xe(1:2);
                uhatVelMatch=uhatVelMatch*(dxe'*Seva.Jparams.Q(7:8,7:8)*dxe)+unit_vector(dxt)*(dxt'*Rt_localE.Qpur*dxt);
            else
                [uhatVelMatch,states] = vmRGVO_tune(xp,xe,gameState.uMaxP,2,gameState.dt,uEvaEst,1,Rt_localE);
                uhatVelMatch=unit_vector(uhatVelMatch); dxe=xp-xe; dxe=dxe(1:2);
                dxt = Rt_localE.xTarget-xe(1:2);
                uhatVelMatch=uhatVelMatch*(dxe'*Seva.Jparams.Q(7:8,7:8)*dxe)+unit_vector(dxt)*(dxt'*Rt_localE.Qpur*dxt);
            end
        else
            [uhatVelMatch,states] = vmRGVO_tune(xp,xe,gameState.uMaxP,2,gameState.dt,uEvaEst,1);
        end
        %         [uhatVelMatch,states] = vmRGVO_tune(xp,xe,gameState.uMaxP,2,gameState.dt,uEvaEst,1);
        uhatVelMatch = unit_vector(uhatVelMatch);
        %limit vector to quadrants 1+4
        theta=atan2(uhatVelMatch(2),uhatVelMatch(1));
        if theta>pi/2 && theta<3*pi/2
            uhatVelMatch=-uhatVelMatch;
        end
        xEt=states.xEout;
        uEtilde=uhatVelMatch*norm(Seva.uMat{iL}(:,1));
        xd = stateE(:,1); xd(9)=0;
        uE = quadControllerACCONLY(xd, zeros(4,1), 3, [uEtilde;0],0);
    elseif strcmp(Seva.controlType,'gt_overx')
        dx = Seva.uMat{iL}(:,1);
        des = stateE(:,1);
        des(7:8) = des(7:8)+dx;
        [uE, validNum] = quadController(stateE(:,1),zeros(4,1),zeros(3,1),des,1,zeros(12,1)); %#ok
    elseif strcmp(Seva.controlType,'gt_overu')
        uE=Seva.uMat{iL}(:,1);
    else
        error('Failed to present a valid gameState.controlType');
    end
    
    %P
    if isfield(gameState,'tryNN')
        if gameState.tryNN
            stateE(:,2) = predict(gameState.NN,[stateE(:,1);uE]);
        else
            stateE(:,2) = feval(Spur.fname,stateE(:,1),uE,gameState.dt,zeros(6,1));
        end
    else
        stateE(:,2) = feval(Spur.fname,stateE(:,1),uE,gameState.dt,zeros(6,1));
    end
    uEm=[uEm uE];
    for ij=1:nmodP
        uEcell{ij,iL} = uEm;
        xEvaCell{ij,iL} = stateE;
        if gameState.kMax>1
            stateEcell{ij,iL}=stateE;
        end
    end
end

%For both, dt>1
if gameState.kMax>1
    for ij=1:nmodP
        for iL=1:nmodE
            stateP=statePcell{ij,iL};
            stateE=stateEcell{ij,iL};
            uPm=[];
            uEm=[];
            for ik=2:gameState.kMax
                xp=stateP([7 8 10 11],ik);
                xe=stateE([7 8 10 11],ik);
                if strcmp(Spur.controlType,'vmquad')
                    if isfield(gameState,'Rtarget')
                        if gameState.Rtarget.useNoHeuristics && ~gameState.Rtarget.useMotionPrediction
                            [uhatVelMatch,states] = vmRGVO_tune(xp,xe,gameState.uMaxP,2,gameState.dt,zeros(2,1),1);
                        elseif gameState.Rtarget.useNoHeuristics && gameState.Rtarget.useMotionPrediction
                            [uhatVelMatch,states] = vmRGVO_tune(xp,xe,gameState.uMaxP,2,gameState.dt,uEvaEst,1,Rt_localE);
                            uhatVelMatch=unit_vector(uhatVelMatch);
                        elseif ~gameState.Rtarget.useNoHeuristics && ~gameState.Rtarget.useMotionPrediction
                            [uhatVelMatch,states] = vmRGVO_tune(xp,xe,gameState.uMaxP,2,gameState.dt,zeros(2,1),1);
                            uhatVelMatch=unit_vector(uhatVelMatch); dxe=xp-xe; dxe=dxe(1:2);
                            dxt = Rt_localE.xTarget-xe(1:2);
                            uhatVelMatch=uhatVelMatch*(dxe'*Seva.Jparams.Q(7:8,7:8)*dxe)+unit_vector(dxt)*(dxt'*Rt_localE.Qpur*dxt);
                        else
                            [uhatVelMatch,states] = vmRGVO_tune(xp,xe,gameState.uMaxP,2,gameState.dt,uEvaEst,1,Rt_localE);
                            uhatVelMatch=unit_vector(uhatVelMatch); dxe=xp-xe; dxe=dxe(1:2);
                            dxt = Rt_localE.xTarget-xe(1:2);
                            uhatVelMatch=uhatVelMatch*(dxe'*Seva.Jparams.Q(7:8,7:8)*dxe)+unit_vector(dxt)*(dxt'*Rt_localE.Qpur*dxt);
                        end
                    else
                        [uhatVelMatch,states] = vmRGVO_tune(xp,xe,gameState.uMaxP,2,gameState.dt,uEvaEst,1);
                    end
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
                elseif strcmp(Spur.controlType,'gt_overu')
                    uP=Spur.uMat{ij}(:,ik);
                else
                    error('Failed to present a valid gameState.controlType');
                end
                
                if strcmp(Seva.controlType,'vmquad')
                    if isfield(gameState,'Rtarget')
                        if gameState.Rtarget.useNoHeuristics && ~gameState.Rtarget.useMotionPrediction
                            [uhatVelMatch,states] = vmRGVO_tune(xp,xe,gameState.uMaxP,2,gameState.dt,zeros(2,1),1);
                        elseif gameState.Rtarget.useNoHeuristics && gameState.Rtarget.useMotionPrediction
                            [uhatVelMatch,states] = vmRGVO_tune(xp,xe,gameState.uMaxP,2,gameState.dt,uEvaEst,1,Rt_localE);
                            uhatVelMatch=unit_vector(uhatVelMatch);
                        elseif ~gameState.Rtarget.useNoHeuristics && ~gameState.Rtarget.useMotionPrediction
                            [uhatVelMatch,states] = vmRGVO_tune(xp,xe,gameState.uMaxP,2,gameState.dt,zeros(2,1),1);
                            uhatVelMatch=unit_vector(uhatVelMatch); dxe=xp-xe; dxe=dxe(1:2);
                            dxt = Rt_localE.xTarget-xe(1:2);
                            uhatVelMatch=uhatVelMatch*(dxe'*Seva.Jparams.Q(7:8,7:8)*dxe)+unit_vector(dxt)*(dxt'*Rt_localE.Qpur*dxt);
                        else
                            [uhatVelMatch,states] = vmRGVO_tune(xp,xe,gameState.uMaxP,2,gameState.dt,uEvaEst,1,Rt_localE);
                            uhatVelMatch=unit_vector(uhatVelMatch); dxe=xp-xe; dxe=dxe(1:2);
                            dxt = Rt_localE.xTarget-xe(1:2);
                            uhatVelMatch=uhatVelMatch*(dxe'*Seva.Jparams.Q(7:8,7:8)*dxe)+unit_vector(dxt)*(dxt'*Rt_localE.Qpur*dxt);
                        end
                    else
                        [uhatVelMatch,states] = vmRGVO_tune(xp,xe,gameState.uMaxP,2,gameState.dt,uEvaEst,1);
                    end
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
                elseif strcmp(Seva.controlType,'gt_overu')
                    uE=Seva.uMat{ij}(:,ik);
                else
                    error('Failed to present a valid gameState.controlType');
                end
                
                %P
                if isfield(gameState,'tryNN')
                    if gameState.tryNN
                        stateP(:,ik+1) = predict(gameState.NN,[stateP(:,ik);uP]);
                        stateE(:,ik+1) = predict(gameState.NN,[stateE(:,ik);uE]);
                    else
                        stateP(:,ik+1) = feval(Spur.fname,stateP(:,ik),uP,gameState.dt,zeros(6,1));
                        stateE(:,ik+1) = feval(Spur.fname,stateE(:,ik),uE,gameState.dt,zeros(6,1));
                    end
                else
                    stateP(:,ik+1) = feval(Spur.fname,stateP(:,ik),uP,gameState.dt,zeros(6,1));
                    stateE(:,ik+1) = feval(Spur.fname,stateE(:,ik),uE,gameState.dt,zeros(6,1));
                end
                uPm=[uPm uP];
                uEm=[uEm uE];
            end
            uPcell{ij,iL} = uPm;
            uEcell{ij,iL} = uEm;
            xPurCell{ij,iL} = stateP;
            xEvaCell{ij,iL} = stateE;
        end
    end
end


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

