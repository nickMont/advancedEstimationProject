function [numExceptions] = testNI_permute(Spur,Seva,gameState,Pk,f_dynname,iter,thresh)

if nargin<=4
    iter=50;
end
if nargin<=5
    thresh=1e-5;
end

[gameCenterR,gameCenterC] = feval(f_dynname,Spur,Seva,gameState);
gameStateTilde=gameState;

numExceptions=0;

for ik=1:iter
    gameStateTilde.xPur=gameState.xPur+chol(Pk.pur)*randn(length(gameState.xPur),1);
    gameStateTilde.xEva=gameState.xEva+chol(Pk.eva)*randn(length(gameState.xEva),1);
    [localR,localC] = feval(f_dynname,Spur,Seva,gameStateTilde);
    if abs(dot(localR,localR)-dot(gameCenterR,gameCenterR)) > thresh || ...
            abs(dot(localC,localC)-dot(gameCenterC,gameCenterC))
        numExceptions=numExceptions+1;
    end
end

end

