function [Q1,Q2,Q3,Q4] = QvecToMats(qVec,nX)
q1Evec=qVec(1:1*nX^2);
q2Evec=qVec(1*nX^2+1:2*nX^2);
q3Evec=qVec(2*nX^2+1:3*nX^2);
q4Evec=qVec(3*nX^2+1:4*nX^2);
Q1=reshape(q1Evec,[nX,nX]);
Q2=reshape(q2Evec,[nX,nX]);
Q3=reshape(q3Evec,[nX,nX]);
Q4=reshape(q4Evec,[nX,nX]);
end

