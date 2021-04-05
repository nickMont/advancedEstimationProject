function p1dot = odeKumarP1(t,pVec,gameStateVals)
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

%Efficiency
invV1=inv(V1);
invV2=inv(V2);
invR1=inv(R11);
invR2=inv(R22);

%Repackage qVec
p1Vec=pVec(1:1*nX^2);
P1=reshape(p1Vec,[nX,nX]);

P1dot = F*P1+P1*F'-P1*H1'*invV1*H1*P1-P1*H2'*invV2*H2*P1+W;

p1dot=reshape(P1dot,[nX^2,1]);

end