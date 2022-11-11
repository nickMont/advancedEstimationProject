Ue2=zeros(nU,nThresh);

x0=-ewxvPur(7:12)+ewxvEva(7:12);

for iiBR=1:5
    % fix Eva, solve Pur
    uOppApprx = Ue2;

    D=[];
    lX=length(x0);
    G=[Bpur zeros(lX,nUlin*(nThresh-1))];
    D=Beva*uOppApprx(:,1);
    if nThresh>1
        for ij=2:nThresh
            D=[D
                A*D(end-lX,:)+Beva*uOppApprx(:,ij)];
            G=[G
                A*G(end-lX,:)];
            G(ij*lX:(ij+1)*lX-1 , ij*nU:(ij+1)*nU-1)=Bpur;
        end
    end

    M=D*x0+T;
    
    % Form block diagonal stacked weight matrices
    Ar = repmat(Seva.Qlin, 1, nThresh);
    Ac = mat2cell(Ar, size(Seva.Qlin,1), repmat(size(Seva.Qlin,2),1,nThresh));
    Qprime = blkdiag(Ac{:});
    Qprime(end-lX+1:end,end-lX+1:end)=Qprime(end-lX+1:end,end-lX+1:end)+Seva.QfLin;
    Ar = repmat(Seva.Rselflin, 1, nThresh);
    Ac = mat2cell(Ar, size(Seva.Rselflin,1), repmat(size(Seva.Rselflin,2),1,nThresh));
    Rprime = blkdiag(Ac{:});

    % solve LS here
    Upp=-inv(G'*Qprime*G + Rprime)*(G'*Q*M);
    Up2=reshape(Upp,[nUlin,nThresh]);


    % fix Pur, solve Eva
    uOppApprx = Up2;
    D=[];
    lX=length(x0);
    G=[Beva zeros(lX,nU*(nThresh-1))];
    D=Bpur*uOppApprx(:,1);
    if nThresh>1
        for ij=2:nThresh
            D=[D
                A*D(end-lX)+Bpur*uOppApprx(:,ij)];
            G=[G
                A*G(end-lX,:)];
            G(ij*lX:(ij+1)*lX-1 , ij*nU:(ij+1)*nU-1)=Beva;
        end
    end

    M=D*x0+T;

    % Form block diagonal stacked weight matrices
    Ar = repmat(Spur.Qlin, 1, nThresh);
    Ac = mat2cell(Ar, size(Spur.Qlin,1), repmat(size(Spur.Qlin,2),1,nThresh));
    Qprime = blkdiag(Ac{:});
    Qprime(end-lX+1:end,end-lX+1:end)=Qprime(end-lX+1:end,end-lX+1:end)+Spur.QfLin;
    Ar = repmat(Seva.Rselflin, 1, nThresh);
    Ac = mat2cell(Ar, size(Spur.Rselflin,1), repmat(size(Spur.Rselflin,2),1,nThresh));
    Rprime = blkdiag(Ac{:});

    % solve LS here
    Uee=-inv(G'*Qprime*G + Rprime)*(G'*Qprime*M);
    Ue2=reshape(Uee,[nU,nThresh]);






end
