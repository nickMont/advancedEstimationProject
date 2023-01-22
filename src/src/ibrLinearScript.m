nU=3;
nThresh=3;
Ue2=zeros(nU,nThresh);

x0=ewxvPur(7:12)-ewxvEva(7:12);

QfP=zeros(6,6);
QfE=zeros(6,6);

QfP(1:3,1:3)=QfP(1:3,1:3)+Spur.Qlin;
QfE(1:3,1:3)=QfE(1:3,1:3)+Seva.Qlin;
QfE=QfE; %accounting for signage difference in inputs

QlinPfull=zeros(6,6);
QlinPfull(1:3,1:3)=Spur.Qlin;

QlinEfull=zeros(6,6);
QlinEfull(1:3,1:3)=Seva.Qlin;
QlinEfull=QlinEfull;

for iiBR=1:5
    % fix Eva, solve Pur
    uOppApprx = Ue2;

    D=[];
    lX=length(x0);
    G=[Bpur zeros(lX,nUlin*(nThresh-1))];
    D=Beva*uOppApprx(:,1);
    H=A;
    if nThresh>1
        for ibt=2:nThresh
            D=[D
                A*D(end-lX+1:end)+Beva*uOppApprx(:,ibt)];
            G=[G
                A*G(end-lX+1:end,1:end)];
            H=[H
                mpower(A,ibt)];
            G((ibt-1)*lX+1:ibt*lX , (ibt-1)*nU+1:ibt*nU)=Bpur;
        end
    end

    M=H*x0+D;
    
    % Form block diagonal stacked weight matrices
    Ar = repmat(QlinPfull, 1, nThresh);
    Ac = mat2cell(Ar, size(QlinEfull,1), repmat(size(QlinPfull,2),1,nThresh));
    Qprime = blkdiag(Ac{:});
    Qprime(end-lX+1:end,end-lX+1:end)=Qprime(end-lX+1:end,end-lX+1:end)+QfE;
    Ar = repmat(Seva.Rselflin, 1, nThresh);
    Ac = mat2cell(Ar, size(Spur.Rselflin,1), repmat(size(Spur.Rselflin,2),1,nThresh));
    Rprime = blkdiag(Ac{:});

    % solve LS here
    Upp=-inv(G'*Qprime*G + Rprime)*(G'*Qprime*M);
    Up2=reshape(Upp,[nUlin,nThresh]);


    % fix Pur, solve Eva
    uOppApprx = Up2;
    D=[];
    lX=length(x0);
    G=[Beva zeros(lX,nU*(nThresh-1))];
    D=Bpur*uOppApprx(:,1);
    H=A;
    if nThresh>1
        for ibt=2:nThresh
            D=[D
                A*D(end-lX+1:end)+Beva*uOppApprx(:,ibt)];
            G=[G
                A*G(end-lX+1:end,1:end)];
            H=[H
                mpower(A,ibt)];
            G((ibt-1)*lX+1:ibt*lX , (ibt-1)*nU+1:ibt*nU)=Beva;
        end
    end

    M=H*x0+D;

    % Form block diagonal stacked weight matrices
    Ar = repmat(QlinEfull, 1, nThresh);
    Ac = mat2cell(Ar, size(QlinEfull,1), repmat(size(QlinEfull,2),1,nThresh));
    Qprime = blkdiag(Ac{:});
    Qprime(end-lX+1:end,end-lX+1:end)=Qprime(end-lX+1:end,end-lX+1:end)+QfP;
    Ar = repmat(Seva.Rselflin, 1, nThresh);
    Ac = mat2cell(Ar, size(Seva.Rselflin,1), repmat(size(Seva.Rselflin,2),1,nThresh));
    Rprime = blkdiag(Ac{:});

    % solve LS here
    Uee=-inv(G'*Qprime*G + Rprime)*(G'*Qprime*M);
    Ue2=reshape(Uee,[nU,nThresh]);






end


xd = xPur(13:24); xd(9)=0;
uEvaQuadOut = quadControllerACCONLY(xd, zeros(4,1), 3, [Ue2(:,1);0],0);
xd = xPur(1:12); xd(9)=0;
uPurQuadOut = quadControllerACCONLY(xd, zeros(4,1), 3, [Up2(:,1);0],0);


% Ueout=Ue2
% Upout=Up2




