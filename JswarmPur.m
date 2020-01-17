function J = JswarmPur(xPurCell,xEvaCell,uMatP,uMatE,Jparams,gameState,inactiveSetP,inactiveSetE)

%note: gameState.Qx is quadratic cost INDEXED OVER EVADERS

J=0;
%Reserve reward
J=J-inactiveSetP*Jparams.reserveRewardWeightPur;
nx=gameState.nx;
nv=nx/2;

for ik=1:gameState.tmax
    %proximity cost
    for iE=1:gameState.numE
        mindist=inf;
        mindex=0;
        if ~isMemberOfInactiveSet(iE,inactiveSetE) %if not in evader reserve
            for iP=1:gameState.numP
                if ~isMemberOfInactiveSet(iP,inactiveSetP) %if not in pursuer reserve
                    thisPurX=xPurCell(:,iP,ik+1);
                    thisEvaX=xEvaCell(:,gameState.numP+iE,ik+1);
                    this_dist=norm(thisPurX(1:nx/2)-thisEvaX(1:nx/2));
                end
                
                %if closest to this evader
                if this_dist<mindist
                    mindist=this_dist;
                    mindex=iP;
                end
            end
            closestActivePur=xPurCell(:,mindex,ik+1);
            dx=closestActivePur-thisEvaX;
            J=J+dx'*Jparams.QxPur(:,:,iE)*dx;
        end
    end
    
    %fuel cost
    for iP=1:length(xPurcell)
        if ~isMemberOfInactiveSet(iP,inactiveSetP) %if not in reserve
            uloc=uMatP(:,iP,ik);
            J=J+uloc'*gameState.Rp(:,:,iP)*uloc;
        end
    end
end

%P_hit, of final target
for iE=1:gameState.numE
    if ~isMemberOfInactiveSet(iE,inactiveSetE)
        targetIndex = gameState.targetIndexList{iE};
        targetDest = gameState.targetLocation{targetIndex};
        thisEvaX = xEvaCell(:,gameState.numP+iE,ik+1);
        dx = targetDest(1:nx-nv)-thisEvaX(1:nx-nv);
        distanceToTarget = norm(dx);
        distanceToTargetAdj = saturationF(distanceToTarget,0,gameState.maxDistanceToTargetPenalized);
        targetWeight=distanceToTargetAdj/gameState.maxDistanceToTargetPenalized;
        J=J+Jparams.targetCost{targetIndex}*targetWeight;
    end
end


end

