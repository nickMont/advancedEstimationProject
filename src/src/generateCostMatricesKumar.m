function [Jpur,Jeva,uP,uE] = generateCostMatricesKumar(Spur, Seva, gameState,vk)
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

% if ~isfield(Spur,'UseVelMatch') || ~isfield(Seva,'UseVelMatch')
%     error('HARDCODED TO ONLY HAND VM/GT HYBRID--Have you added a UseVelMatch flag to Spur/Seva?')
% elseif ~Spur.UseVelMatch || ~Seva.UseVelMatch
%     error('UseVelMatch set to false but generateCostMatricesVM2 was called')
% end

nmodP = length(Spur.uMat);
nmodE = length(Seva.uMat);

Jpur=9001*ones(nmodP,nmodE);
Jeva=42*ones(nmodP,nmodE);

nx = length(gameState.x0);
% state = zeros(nx,gameState.kMax+1);

Jpur=zeros(nmodP,nmodE);
Jeva=zeros(nmodP,nmodE);

%For both, dt>1
for iP=1:nmodP
    pcTC = iP/nmodP*100
    for iE=1:nmodE
        uPm=Spur.uMat{iP};
        uEm=Seva.uMat{iE};
        xH=[];
        tH=[];
        u1=[];
        u2=[];
        paramsTemp=gameState.params;
        xT = gameState.x0;
        for ik=1:gameState.kmax
            upp = uPm(:,ik);
            uee = uEm(:,ik);
            tryLin = false;
            if isfield(gameState,'tryLinearPropagation')
                if gameState.tryLinearPropogatation
                    tryLin=true;
                end
            end
            if tryLin
                xT = gameState.A*xT + gameState.Bstack*[upp;uee];
            else
                paramsTemp.u1=upp;
                paramsTemp.u2=uee;
                os = @(t,x) fdyn_kumarSamp(t,x,paramsTemp);
                tv=gameState.t0+[gameState.dt*(ik-1) gameState.dt*ik];
                [t2,x2]=ode45(os,tv,xT);
                xT = (x2(end,:))';
            end
            
            u1=[u1 repmat(upp, [1 length(t2)])];
            u2=[u2 repmat(uee, [1 length(t2)])];
            xH=[xH;x2];
            tH=[tH;t2];
        end
        %NB: Kumar cost for evader is defined as a value
        Jpur(iP,iE)=evalCostFunc(tH,xH,u1,u2,'j1Int','j1Final',paramsTemp);
        Jeva(iP,iE)=-evalCostFunc(tH,xH,u1,u2,'j2Int','j2Final',paramsTemp);
    end
end


uP=Spur.uMat;
uE=Seva.uMat;

% Jpur
% Jeva

% noiseCostPur=3409;
% noiseCostEva=3409;
% 
% for iP=1:nmodP
%     for iE=1:nmodE
%         
%         %NB: Kumar cost for evader is defined as a value
%         
%     end
% end


end


