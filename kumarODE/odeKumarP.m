function pVecDot = odeKumarP(t,pVec,Qvals,Qtimes,gameStateVals)
%Q contains Q.times, Q.values.  Q.times is nTx1, Q.values is nTxnP

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
p1Vec=pVec(1:1*nX^2);
p2Vec=pVec(1*nX^2+1:2*nX^2);
p3Vec=pVec(2*nX^2+1:3*nX^2);
P1=reshape(p1Vec,[nX,nX]);
P2=reshape(p2Vec,[nX,nX]);
P3=reshape(p3Vec,[nX,nX]);

%Efficiency
invV1=inv(V1);
invV2=inv(V2);
invR1=inv(R11);
invR2=inv(R22);

%Interpolate P, linear interpolation
Qq=(interp1(Qtimes,Qvals',t))';
Q1f=Qq(1:1*nX^2);
Q2f=Qq(1*nX^2+1:2*nX^2);
Q3f=Qq(2*nX^2+1:3*nX^2);
Q4f=Qq(3*nX^2+1:4*nX^2);
Q1=reshape(Q1f,[nX,nX]);
Q2=reshape(Q2f,[nX,nX]);
Q3=reshape(Q3f,[nX,nX]);
Q4=reshape(Q4f,[nX,nX]);

% % % The ODEs themselves
%(12)
P1dot = F*P1+P1*F'-P1*H1'*invV1*H1*P1-P1*H2'*invV2*H2*P1+W;
%(14)
P2dot = F*P2+P2*F'-P2*H2'*invV2*H2*P2-G1*invR1*G1'*(Q1+Q3)*(P2-P3)-(P2-P3')*(Q1+Q3)'*G1*invR1*G1'+W;
%(16)
P3dot = F*P3+P3*F'-P1*H1'*invV1*H1*P3-P1*H2'*invV2*H2*P3+P1*(Q1+Q3)'*G1*invR1*G1'-P3*(Q1+Q3)'*G1*invR1*G1'+...
    -P3*H2'*invV2*H2*P2+P1*H2'*invV2*H2*P2+W;

%Package to exit
p1dot=reshape(P1dot,[nX^2,1]);
p2dot=reshape(P2dot,[nX^2,1]);
p3dot=reshape(P3dot,[nX^2,1]);

pVecDot=[p1dot;p2dot;p3dot];

end

