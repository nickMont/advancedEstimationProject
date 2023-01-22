nUbr=2;
nUlinbr=nUbr;
nThreshbr=3;
Ue2br=zeros(nUbr,nThreshbr);

Abr=eye(4);
Abr(1:2,3:4) =[dt 0
             0 dt];
Bbr=[dt^2/2 0 
    0 dt^2/2 
    dt 0 
    0 dt];
Bevabr=Bbr;
Bpurbr=Bbr;


twoDind=[7 8 10 11];
x0br=ewxvPur([7 8 10 11])-ewxvEva([7 8 10 11]);

QfPbr=zeros(4,4);
QfEbr=zeros(4,4);

QfPbr=QpurPM;
QfEbr=QevaPM;
QfEbr=QfEbr; %accounting for signage difference in inputs

QlinPfullBR=Qpur(twoDind,twoDind);

QlinEfullBR=Qeva(twoDind,twoDind);

for iiBR=1:5
    % fix Eva, solve Pur
    uOppApprxbr = Ue2br;

    Dbr=[];
    lXbr=length(x0br);
    Gbr=[Bpurbr zeros(lXbr,nUlinbr*(nThreshbr-1))];
    Dbr=Bevabr*uOppApprxbr(:,1);
    Hbr=Abr;
    if nThreshbr>1
        for ibt=2:nThreshbr
            Dbr=[Dbr
                Abr*Dbr(end-lXbr+1:end)+Bevabr*uOppApprxbr(:,ibt)];
            Gbr=[Gbr
                Abr*Gbr(end-lXbr+1:end,1:end)];
            Hbr=[Hbr
                mpower(Abr,ibt)];
            Gbr((ibt-1)*lXbr+1:ibt*lXbr , (ibt-1)*nUbr+1:ibt*nUbr)=Bpurbr;
        end
    end

    M=Hbr*x0br+Dbr;
    
    % Form block diagonal stacked weight matrices
    Arbr = repmat(QlinPfullBR, 1, nThreshbr);
    Acbr = mat2cell(Arbr, size(QlinEfullBR,1), repmat(size(QlinPfullBR,2),1,nThreshbr));
    Qprimebr = blkdiag(Acbr{:});
    Qprimebr(end-lXbr+1:end,end-lXbr+1:end)=Qprimebr(end-lXbr+1:end,end-lXbr+1:end)+QfEbr;
    Arbr = repmat(Seva.Rselflin, 1, nThreshbr);
    Acbr = mat2cell(Arbr, size(Spur.Rselflin,1), repmat(size(Spur.Rselflin,2),1,nThreshbr));
    Rprimebr = blkdiag(Acbr{:});

    % solve LS here
    Uppbr=-inv(Gbr'*Qprimebr*Gbr + Rprimebr)*(Gbr'*Qprimebr*M);
    Up2br=reshape(Uppbr,[nUlinbr,nThreshbr]);


    % fix Pur, solve Eva
    uOppApprxbr = Up2br;
    Dbr=[];
    lXbr=length(x0br);
    Gbr=[Bevabr zeros(lXbr,nUbr*(nThreshbr-1))];
    Dbr=Bpurbr*uOppApprxbr(:,1);
    Hbr=Abr;
    if nThreshbr>1
        for ibt=2:nThreshbr
            Dbr=[Dbr
                Abr*Dbr(end-lXbr+1:end)+Bevabr*uOppApprxbr(:,ibt)];
            Gbr=[Gbr
                Abr*Gbr(end-lXbr+1:end,1:end)];
            Hbr=[Hbr
                mpower(Abr,ibt)];
            Gbr((ibt-1)*lXbr+1:ibt*lXbr , (ibt-1)*nUbr+1:ibt*nUbr)=Bevabr;
        end
    end

    M=Hbr*x0br+Dbr;

    % Form block diagonal stacked weight matrices
    Arbr = repmat(QlinEfullBR, 1, nThreshbr);
    Acbr = mat2cell(Arbr, size(QlinEfullBR,1), repmat(size(QlinEfullBR,2),1,nThreshbr));
    Qprimebr = blkdiag(Acbr{:});
    Qprimebr(end-lXbr+1:end,end-lXbr+1:end)=Qprimebr(end-lXbr+1:end,end-lXbr+1:end)+QfPbr;
    Arbr = repmat(Seva.Rselflin, 1, nThreshbr);
    Acbr = mat2cell(Arbr, size(Seva.Rselflin,1), repmat(size(Seva.Rselflin,2),1,nThreshbr));
    Rprimebr = blkdiag(Acbr{:});

    % solve LS here
    Ueebr=-inv(Gbr'*Qprimebr*Gbr + Rprimebr)*(Gbr'*Qprimebr*M);
    Ue2br=reshape(Ueebr,[nUbr,nThreshbr]);



end


xd = xPur(13:24); xd(9)=0;
uEvaQuadOut = quadControllerACCONLY(xd, zeros(4,1), 3, [Ue2br(:,1);0],0);
xd = xPur(1:12); xd(9)=0;
uPurQuadOut = quadControllerACCONLY(xd, zeros(4,1), 3, [Up2br(:,1);0],0);


% Ueout=Ue2
% Upout=Up2




