function [uPur,uEva,outputflag,uValP,uValE,Sminimax,Smisc] = f_dyn2(Spur,Seva,gameState,vk,miscParams)
if nargin<=3
    vk=zeros(100,1);
end
lenv=floor(length(vk)/2);
Sminimax=[];

%Generate cost matrices, determine whether to use robust expectation or not
generateUsingUT=false;
if nargin>=5
    if isfield(miscParams,'useUT')
        if miscParams.useUT
            generateUsingUT=true;
        end
    end
end

%Robust expectation
if generateUsingUT
    SpurTemp=Spur;
    SevaTemp=Seva;
    gameStateTemp=gameState;
    xk = [gameState.xPur;gameState.xEva];
    nx = length(xk);
    nv=0; %process noise ignored for now
    Sk_aug = chol(miscParams.Qk)';
    alpha = 1e-3;
    beta = 2;
    kappa = 0;
    lambda_p = alpha^2*(kappa + nx + nv) - nx - nv;
    c_p = sqrt(nx+nv+lambda_p);
    w_mean_center = lambda_p/(nx + nv + lambda_p);
    w_mean_reg = 1/(2*(nx + nv + lambda_p));
    [Cp0,Ce0,uP0,uE0] = generateCostMatricesVMquad(SpurTemp,SevaTemp,gameStateTemp);
    CpSum = Cp0*w_mean_center;
    CeSum = Ce0*w_mean_center;
%     uPsum = uP0*w_mean_center; %note: need to convert cell2mat
%     uEsum = uE0*w_mean_center;
    sgn = 1;
    for ij=1:2*nx
        colno = mod(ij,nx)+1;
        if(ij > (nx))
            sgn = -1;
        end
        xaug_ij = xk + sgn*c_p*Sk_aug(:,colno);
        gameStateTemp.xPur=xaug_ij(1:nx/2);
        gameStateTemp.xEva=xaug_ij(nx/2+1:nx);
        [Cp1,Ce1,uP1,uE1] = generateCostMatricesVMquad(SpurTemp,SevaTemp,gameStateTemp);
        CpSum = CpSum + Cp1*w_mean_reg;
        CeSum = CeSum + Ce1*w_mean_reg;
%         uPsum = uPsum + uP1*w_mean_reg;
%         uEsum = uEsum + uE1*w_mean_reg;
    end
    Cpur=CpSum;
    Ceva=CeSum;
%     uP=uPsum;
%     uE=uEsum;
    uP=uP0;
    uE=uE0;
    [nP,nE]=size(Cpur);
elseif ~generateUsingUT
    [Cpur,Ceva,uP,uE]=generateCostMatricesVMquad(Spur,Seva,gameState);
    [nP,nE]=size(Cpur);
end
    
[indminimax,~,~]=minimax2(Cpur,Ceva);
Sminimax.index=indminimax;
Sminimax.uP=uP{indminimax(1),indminimax(2)};
Sminimax.uE=uE{indminimax(1),indminimax(2)};

%aa=LH2(-Cpur,-Ceva)
%lhPur=aa{1}
%lhEva=aa{2}
%Solve Nash
[rdeq,flag]=findRDEq(-Cpur,-Ceva);
outputflag=flag;

if flag==0 %if no unique solution, run LH2 and take E(u) for result
    sol=LH2(-Cpur,-Ceva);
    %     uPur = zeros(gameState.nu,1);
    %     uEva = zeros(gameState.nu,1);
    %     for ij=1:length(sol{1})
    %         uPur=uPur + sol{1}(ij)*Spur.uMat{ij};
    %     end
    %     for ij=1:length(sol{2})
    %         uEva = uEva + sol{2}(ij)*Seva.uMat{ij};
    %     end
    uPur=sol{1};
    uEva=sol{2};
    [~,indexP]=max(uPur);
    [~,indexE]=max(uEva);
    uValP=[];
    uValE=[];
    if ~(isempty(uP))
        uInd=randsample(nP,1,true,uPur);
        uValP=uP{uInd};
        uInd=randsample(nE,1,true,uEva);
        uValE=uE{uInd};
    end
    Smisc.uPair=[sol{1};sol{2}];
    Smisc.uPairMax=[indexP;indexE];
else %unique solution found
    up_index=rdeq(1,1);
    ue_index=rdeq(2,1);
    uPur=Spur.uMat{up_index};
    uEva=Seva.uMat{ue_index};
    uValP=uP{up_index,ue_index};
    uValE=uE{up_index,ue_index};
    Smisc.uPair=[up_index;ue_index];
    Smisc.uPairMax=Smisc.uPair;
end

end

