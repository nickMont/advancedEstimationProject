function xdot = dynamicsKumar(t,x,Qvals,Qtimes,A,B,gameStateVals)

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

xPlayer=x(1:nX);
xOpponent=x(nX+1:end);

Qq=(interp1(Qtimes,Qvals',t))';
Q1f=Qq(1:1*nX^2);
Q2f=Qq(1*nX^2+1:2*nX^2);
Q3f=Qq(2*nX^2+1:3*nX^2);
Q4f=Qq(3*nX^2+1:4*nX^2);
Q1=reshape(Q1f,[nX,nX]);
Q2=reshape(Q2f,[nX,nX]);
Q3=reshape(Q3f,[nX,nX]);
Q4=reshape(Q4f,[nX,nX]);

u = -R22*G2'*Q2*(xPlayer-xOpponent);
if norm(u)>gameStateVals.umax
    u=gameStateVals.umax*u/norm(u);
end

xPlayerDot = A*xPlayer+B*u;
xOpponentDot = A*xOpponent; %approximate estimate for opponent's behavior
xdot = [xPlayerDot;xOpponentDot];

end

