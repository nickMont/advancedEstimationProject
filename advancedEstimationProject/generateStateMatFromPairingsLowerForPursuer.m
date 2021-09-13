function [xCell,uCell] = generateStateMatFromPairingsLowerForPursuer(pairings, Spur, Seva, gameState)
nx = length(gameState.xPur{1});
nv = nx/2;
nu = nv;
numP = gameState.numPursuers;
numE = gameState.numEvaders;
state = zeros(nx,numP+numE,gameState.kMax+1);
uMat = zeros(nu,numP+numE,gameState.kMax);
nmod = length(pairings);
xCell = cell(nmod,1);
uCell = cell(nmod,1);
for ij=1:nmod
    state = zeros(nx,numP+numE,gameState.kMax+1);
    uMat = zeros(nu,numP+numE,gameState.kMax);
    nP=length(pairings{ij}{1});
    for iL=1:nP
        this_numP=length(pairings{ij}{1}{iL});
    
        %set up initial state
        ministate=[];
        for iM=1:size(pairings{ij}{1}{iL})
            pairval=pairings{ij}{1}{iL};
            purIndex=pairval(iM);
            ministate=[ministate; gameState.xPur{purIndex}];
            state(:,purIndex,1)=gameState.xPur{purIndex};
        end
        for iM=1:size(pairings{ij}{2}{iL})
            pairval=pairings{ij}{2}{iL}(iM);
            evaIndex=pairval(iM);
            ministate=[ministate; gameState.xEva{evaIndex}];
            state(:,numP+evaIndex,1)=gameState.xEva{evaIndex};
        end
%         Qp=Jparams.lowlevel.Q;
%         Qe=Jparams.lowlevel.Q;
%         Rp=Jparams.lowlevel.R;
%         Re=Jparams.lowlevel.R;
        Qp=Spur.Jparams.QxPur(:,:,1);
        Qe=Seva.Jparams.QxEva(:,:,1);
        Rp=Spur.Jparams.Rpur(:,:,1);
        Re=Seva.Jparams.Reva(:,:,1);
        
        if this_numP==1
            qqrr=0.1*[diag(Qp); diag(Qe); diag(Rp); diag(Re)]; %0.1 was tuning param in trainPE
        else
            qqrr=0.1*[diag(Qp); diag(Qp); diag(Qe); diag(Rp); diag(Rp); diag(Re)];
        end          

        %propagate this pairing
        for ik=1:gameState.kMax
            %propate for 1v1
            if this_numP==1
                uVM=vmRGVO_max(ministate(1:nx),ministate(nx+1:end),Spur.umaxP,nx/2,gameState.dt,zeros(2,1));
                heading=uVM/norm(uVM);
                uVec=[Spur.umaxP*heading;-Seva.umaxE*heading];
                ministate(1:nx) = feval(Spur.fname{purIndex},ministate(1:nx),uVec(1:nu),gameState.dt,zeros(nv,1));
                ministate(nx+1:end) = feval(Seva.fname{evaIndex},ministate(nx+1:end),uVec(nu+1:end),gameState.dt,zeros(nv,1));
            else %propagate for Xv1
                %uVec=predict(network2,[ministate;qqrr])';
                for iM=1:2
                    purIndex = pairings{iL}{1}(iM);
                    ministate((iM-1)*nx+1:iM*nx) = feval(Spur.fname{purIndex},ministate((iM-1)*nx+1:iM*nx),uVec(((iM-1)*nu+1:iM*nu)),gameState.dt,zeros(nv,1));
                end
                ministate(2*nx+1:end) = feval(Spur.fname{evaIndex},ministate(2*nx+1:end),uVec(2*nu+1:end),gameState.dt,zeros(nv,1));
            end
            
            %pack into state
            for iM=1:this_numP
                pairval=pairings{ij}{1}{iL};
                ind=pairval(iM);
                state(:,ind,ik+1)=ministate((iM-1)*nx+1:iM*nx);
                uMat(:,ind,ik)=uVec((iM-1)*nu+1:iM*nu);
            end
            pairval=pairings{ij}{2}{iL};
            ind=pairval(iM);
            state(:,numP+ind,ik+1)=ministate(this_numP*nx+1:end);
            uMat(:,numP+ind,ik)=uVec(this_numP*nu+1:end);
        end
    end
    xCell{ij} = state;
    uCell{ij}=uMat;
end

end