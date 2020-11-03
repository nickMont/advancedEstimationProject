function J = JswarmEva(xPurCell,xEvaCell,uMat,Jparams,gameState,inactiveSetP,inactiveSetE)

%note: gameState.Qx is quadratic cost INDEXED OVER EVADERS

J=0;
%Reserve reward
J=J-length(inactiveSetE)*Jparams.reserveRewardWeightEva;
nx=gameState.nx;
nv=nx/2;

for ik=1:gameState.kMax
    %proximity cost
    for iE=1:gameState.numEvaders
        mindist=inf;
        mindex=0;
        if ~isMemberOfInactiveSet(iE,inactiveSetE) %if not in evader reserve
            for iP=1:gameState.numPursuers
                if ~isMemberOfInactiveSet(iP,inactiveSetP) %if not in pursuer reserve
                    thisPurX=xPurCell(:,iP,ik+1);
                    thisEvaX=xEvaCell(:,gameState.numPursuers+iE,ik+1);
                    this_dist=norm(thisPurX(1:nx/2)-thisEvaX(1:nx/2));
                else
                    this_dist=inf;
                end
                
                %if closest to this evader
                if this_dist<mindist
                    mindist=this_dist;
                    mindex=iP;
                end
            end
            closestActivePur=xPurCell(:,mindex,ik+1);
            dx=closestActivePur-thisEvaX;
            J=J-dx'*Jparams.QxEva(:,:,iE)*dx;
        end
    end
    
    %fuel cost
    for iE=1:gameState.numEvaders
        if ~isMemberOfInactiveSet(iE,inactiveSetE) %if not in reserve
            uloc=uMat(:,gameState.numPursuers+iE,ik);
            J=J+uloc'*Jparams.Reva(:,:,iE)*uloc;
        end
    end
end

%P_hit, of final target
for iE=1:gameState.numEvaders
    if ~isMemberOfInactiveSet(iE,inactiveSetE)
        targetIndex = gameState.targetIndexList{iE};
        targetDest = gameState.targetLocation{targetIndex};
        thisEvaX = xEvaCell(:,gameState.numPursuers+iE,ik+1);
        dx = targetDest(1:nx-nv)-thisEvaX(1:nx-nv);
        distanceToTarget = norm(dx);
        distanceToTargetAdj = saturationF(distanceToTarget,0,gameState.maxDistanceToTargetPenalized);
        targetWeight=distanceToTargetAdj/gameState.maxDistanceToTargetPenalized;
        J=J-Jparams.targetValue{targetIndex}*targetWeight;
    end
end


end

