function [xPur,xEva,QEva,REva] = unpackState(xaug)

nx=2; %game is 2D

xPur=xaug(1:2*nx);
xEva=xaug(2*nx+1:4*nx);
Qelements=xaug(4*nx+1:6*nx);
QEva=diag(Qelements);
Relements=xaug(6*nx+1:7*nx);
REva=diag(Relements);
end

