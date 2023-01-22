function [out,inactiveC,inactiveR] = simultaneousCombMatWithInactivev2(C,R)
%assumes that dim(C)==dim(R)
%returns currentMatriR, of size numPairs R dim(C)+dim(R)
%Each row represents a set of pairings, written as
%  C1 R1 C2 R2 C3 R3  etc.
%NOTE: 0 CANNOT appear in C or R

LC=length(C);
LR=length(R);
Lm=max(LC,LR);
allActiveSetsC=[];
allInactiveSetsC=[];
allActiveSetsR=[]; %size can be precomputed but the computational savings are hard to justify
allInactiveSetsR=[];
for ij=1:length(C)
    L=nchoosek(C,ij);
    [m,n]=size(L);
    allActiveSetsC=[allActiveSetsC; L zeros(m,Lm-n)]; %#ok<*AGROW>
%     for ik=1:m
%         thisInactive = setdiff(L(ik,:),C);
%         dd=length(thisInactive);
%         allInactiveSetsC = [allInactiveSetsC; thisInactive zeros(1,LC-dd)];
%     end
end
allActiveSetsC;

%LR=length(R);
%allActiveSetsR=zeros(1,LR);
for ij=1:length(R)
    L=nchoosek(R,ij);
    [m,n]=size(L);
    %thisInactive = setdiff(L,R);
    allActiveSetsR=[allActiveSetsR; L zeros(m,Lm-n)];
end
allActiveSetsR;

[numSetsC,~]=size(allActiveSetsC);
[numSetsR,~]=size(allActiveSetsR);

out=[];
Lo=min(LC,LR)^2;
for ij=1:numSetsC
    for ik=1:numSetsR
        Ctmp=allActiveSetsC(ij,:);
        Ctmp0=Ctmp(Ctmp(:)~=0);
        Rtmp=allActiveSetsR(ik,:);
        Rtmp0=Rtmp(Rtmp(:)~=0);
        if 0==length(Ctmp0)-length(Rtmp0)
            p = perms(Ctmp0);
            [m,n] = size(p);
            combMat = zeros(m,2*n);
            repR = repmat(Rtmp0,m,1);
            combMat(:,2:2:end) = repR;
            combMat(:,1:2:end-1) = p;
            [x,y]=size(combMat);
            if y<Lo
                combMat=[combMat zeros(x,Lo-y)];
            end
            out=[out;combMat];
            %add in inactive mats?
        end
    end
end
out=unique(out,'Rows');

[a,~]=size(out);
inactiveC=zeros(a,LC);
inactiveR=zeros(a,LR);
for ij=1:a
    activeC=out(ij,1:2:end-1);
    this_inactiveC=setdiff(C,activeC);
    this_inactiveC=this_inactiveC(this_inactiveC(:)~=0);
    inactiveC(ij,1:length(this_inactiveC))=this_inactiveC;
    activeR=out(ij,2:2:end);
    this_inactiveR=setdiff(R,activeR);
    this_inactiveR=this_inactiveR(this_inactiveR(:)~=0);
    inactiveR(ij,1:length(this_inactiveR))=this_inactiveR;
end



end

