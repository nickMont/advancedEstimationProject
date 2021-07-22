function [Jpur,Jeva,Jpur2,Jeva2,jset2] = generateCostMatricesHC_closedloop(Spur, Seva, gameState,vk)
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

Jpur=zeros(nmodP,nmodE);
Jeva=zeros(nmodP,nmodE);
tmaxRun=gameState.tMaxSim;
noCapErr = 99; % capture "time" if no capture occurs
captureBonus=-9001;
for iP=1:nmodP
%     pctC=iP/nmodP*100
%     iP
    for iE=1:nmodE        
        uP = Spur.uMat{iP,1};
        uE = Seva.uMat{iE,1};
        uP = [uP; 100+1e-6 0];
        if strcmp(gameState.params.uEType,'step')
            uE = [uE; 100+1e-6 0];
        end
        
        hc_params.mu = gameState.params.mu;
        hc_params.PsiMat = uE;
        hc_params.uPMat = uP;
        hc_params.PsiType = gameState.params.uEType;
        
        HC_ODE = @(t,x) homicidalChauffeurForwardFOH(t,x,hc_params);
        [a,b]=ode45(HC_ODE,[gameState.t0 gameState.tMaxSim],gameState.x0);
        
        l2 = b(:,1).^2 + b(:,2).^2;
        bDot=HC_ODE(tmaxRun,b(end,:)');
        indf = find(l2<=(gameState.params.L)^2);
        %Does capture occur at indf?
        doesCap=false;
        capTime=9001;
        if length(indf)>=1
            for ij=1:length(indf)
                xydot=HC_ODE(a(indf(ij)),b(indf(ij),:)');
                if b(indf(ij),1)/xydot(1)<0 && b(indf(ij),2)/xydot(2)<0
                    doesCap = true;
                    if a(indf(ij))<capTime
                        capTime=a(indf(ij));
                    end
                end
            end
        end
        
        thisJ=90010;
        if doesCap
            thisJ = captureBonus+capTime;
        else
            thisJ = l2(end)+bDot(1)+bDot(2);
            thisJ = sqrt(l2(end));
            thisJ = l2(end);
        end
%         thisJ=-thisJ;
        Jpur(iP,iE) = thisJ;
        Jeva(iP,iE) = -thisJ;
        
    end
end


% noiseCostPur=3409;
% noiseCostEva=3409;
% 
% for iP=1:nmodP
%     for iE=1:nmodE
%         Jpur(iP,iE)=feval(Spur.Jname,xPurCell{iP,iE},xEvaCell{iP,iE},uPcell{iP,iE},uEcell{iP,iE},Spur.Jparams);
%         Jeva(iP,iE)=feval(Seva.Jname,xEvaCell{iP,iE},xPurCell{iP,iE},uEcell{iP,iE},uPcell{iP,iE},Seva.Jparams);
%     end
% end

% Cull nonunique columns
jset2 = [];
Jpur2 = [];
for ij=1:nmodE
    col=Jpur(:,ij);
    if length(unique(col))>1
        jset2 = [jset2 ij];
        Jpur2 = [Jpur2 col];
    end
end
Jeva2 = -Jpur2;


end

