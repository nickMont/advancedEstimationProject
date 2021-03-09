function qVecDot = odeKumarQ(t,qVec,Pvals,Ptimes,gameStateVals)
%NOTE: Run with Tspan = [T 0]. Matlab handles negation internally.

%P contains P.times, P.values.  P.times is nTx1, P.values is nTxnP

%Unpack stuff
nX=gameStateVals.nX;
V1=gameStateVals.V1;
V2=gameStateVals.V2;
W=gameStateVals.W;
R11=gameStateVals.R11;
R12=gameStateVals.R12;
R21=gameStateVals.R21;
R22=gameStateVals.R22;
F=gameStateVals.F;
G1=gameStateVals.G1;
G2=gameStateVals.G2;
H1=gameStateVals.H1;
H2=gameStateVals.H2;

%Repackage qVec
q1Evec=qVec(1:1*nX^2);
q2Evec=qVec(1*nX^2+1:2*nX^2);
q3Evec=qVec(2*nX^2+1:3*nX^2);
q4Evec=qVec(3*nX^2+1:4*nX^2);
Q1=reshape(q1Evec,[nX,nX]);
Q2=reshape(q2Evec,[nX,nX]);
Q3=reshape(q3Evec,[nX,nX]);
Q4=reshape(q4Evec,[nX,nX]);

%Efficiency
invV1=inv(V1);
invV2=inv(V2);
invR1=inv(R11);
invR2=inv(R22);

%Interpolate P, linear interpolation
Pq=(interp1(Ptimes,Pvals',t))';
P1f=Pq(1:1*nX^2);
P2f=Pq(1*nX^2+1:2*nX^2);
P3f=Pq(2*nX^2+1:3*nX^2);
P1=reshape(P1f,[nX,nX]);
P2=reshape(P2f,[nX,nX]);
P3=reshape(P3f,[nX,nX]);

% % % The ODEs themselves
%(21)
Q1dot = -Q1*F-F'*Q1+Q1*G1*invR1*G1'*Q1-Q1*G2*inv(R22)*G2'*Q2+H2'*invV2*H2*P2*Q3'-H2'*invV2*H2*P2*Q4;
%(22)
Q2dot = -Q2*F-F'*Q2-Q2*G2*invR2*G2'*Q2+Q2*G1*invR1*G1'*Q1+Q1'*G1*invR1*G1'*Q2-Q1'*G1*invR1*R21*invR1*G1'*Q1;
%(18)
Q3dot = -Q1dot-(Q1+Q3)*F-F'*(Q1+Q3)+(Q1+Q3)'*G1*invR1*G1'*(Q1+Q3)+Q3*P2*H2'*invV2*H2+H2'*invV2*H2*P2*Q3';
%(24)
Q4dot = (-F'+Q1'*G1*invR1*G1'-Q2*G2*invR2*G2'+H2'*invV2*H2*P2)*Q4+Q4*(-F+G1*invR1*G1'*Q1-G2*invR2*G2'*Q2+P2*H2'*invV2*H2)+...
    +Q3'*G1*invR1*G1'*Q3+Q2*G2*invR2*R12*invR2*G2'*Q2+Q3'*G2*invR2*G2'*Q2+Q2*G2*invR2*G2'*Q3;

% n1q=norm(Q1dot)
% n2q=norm(Q2dot)
% n3q=norm(Q3dot)
% n4q=norm(Q4dot)

%Package to exit
q1dot=reshape(Q1dot,[nX^2,1]);
q2dot=reshape(Q2dot,[nX^2,1]);
q3dot=reshape(Q3dot,[nX^2,1]);
q4dot=reshape(Q4dot,[nX^2,1]);

qVecDot=[q1dot;q2dot;q3dot;q4dot];

end

