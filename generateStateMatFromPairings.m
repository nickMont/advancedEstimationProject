function [xCell,uCell] = generateStateMatFromPairings(pairings, Spur, Seva, gameState, network1, network2)
nx = length(gameState.xPur{1});
nv = nx/2;
nu = nv;
numP = gameState.numPursuers;
numE = gameState.numEvaders;
state = zeros(nx,numP+numE,gameState.kMax+1);
uMat = zeros(nu,numP+numE,gameState.kMax);
nmod = size(pairings);
xCell = cell(nmod);
uCell = cell(nmod);
for ij=1:nmod
    state = zeros(nx,numP+numE,gameState.kMax+1);
    uMat = zeros(nu,numP+numE,gameState.kMax);
    nP=size(pairings{ij});
    for iL=1:nP
        this_numP=length(pairings{ij}{1});
    
        %set up initial state
        ministate=[];
        for iM=1:size(pairings{iL}(1))
            purIndex = pairings{iL}{1}(iM);
            ministate=[ministate; gameState.xPur{purIndex}];
            state(:,purIndex,1)=gameState.xPur{purIndex};
        end
        for iM=1:size(pairings{iL}(2))
            evaIndex=pairings{iL}{2}(iM);
            ministate=[ministate; gameState.xPur{evaIndex}];
            state(:,numP+evaIndex,1)=gameState.xEva{evaIndex};
        end
        Qp=Spur.lowlevel.Q;
        Qe=Seva.lowlevel.Q;
        Rp=Spur.lowlevel.R;
        Re=Seva.lowlevel.R;
        
        if this_numP==1
            qqrr=0.1*[diag(Qp) diag(Qe) diag(Rp) diag(Re)]'; %0.1 was tuning param in trainPE
        else
            qqrr=0.1*[diag(Qp) diag(Qp) diag(Qe) diag(Rp) diag(Rp) diag(Re)]';
        end          

        %propagate this pairing
        for ik=1:gameState.kMax
            %propate for 1v1
            if this_numP==1
                uVec=predict(network1,[ministate;qqrr]);
                ministate(1:nx) = feval(Spur.fname{purIndex},ministate(1:nx),uVec(1:nu),gameState.dt,zeros(nv,1));
                ministate(nx+1:end) = feval(Seva.fname{evaIndex},ministate(nx+1:end),uVec(nu+1:end),gameState.dt,zeros(nv,1));
            else %propagate for Xv1
                uVec=predict(network2,[ministate;qqrr]);
                for iM=1:2
                    purIndex = pairings{iL}{1}(iM);
                    ministate((iM-1)*nx+1:iM*nx) = feval(Spur.fname{purIndex},ministate((iM-1)*nx+1:iM*nx),uVec(((iM-1)*nu+1:iM*nu)),gameState.dt,zeros(nv,1));
                end
                ministate(2*nx+1:end) = feval(Spur.fname{evaIndex},ministate(2*nx+1:end),uVec(2*nu+1:end),gameState.dt,zeros(nv,1));
            end
            
            %pack into state
            for iM=1:this_numP
                state(:,pairings{iL}{1}(iM),ik+1)=ministate((iM-1)*nx+1:iM*nx);
                uMat(:,pairings{iL}{1}(iM),ik)=uVec((iM-1)*nu+1:iM*nu);
            end
            state(:,numP+pairings{iL}{2}(1),ik+1)=ministate(this_numpP*nx+1:end);
            uMat(:,numP+pairings{iL}{2}(1),ik)=uVec(this_numP*nu+1:end);
        end
    end
    xCell{ij} = state;
    uCell=uMat;
end


end