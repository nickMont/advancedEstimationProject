function [P1,P2,P3] = PvecToMats(pVec,nX)
q1Evec=pVec(1:1*nX^2);
q2Evec=pVec(1*nX^2+1:2*nX^2);
q3Evec=pVec(2*nX^2+1:3*nX^2);
P1=reshape(q1Evec,[nX,nX]);
P2=reshape(q2Evec,[nX,nX]);
P3=reshape(q3Evec,[nX,nX]);
end

