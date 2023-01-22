function out = simultaneousCombMatWithInactive(C,R)
%assumes that dim(C)==dim(R)
%returns currentMatriR, of size numPairs R dim(C)+dim(R)
%Each row represents a set of pairings, written as
%  C1 R1 C2 R2 C3 R3  etc.
%NOTE: 0 CANNOT appear in C or R

LC=length(C);
LR=length(R);
Lm=max(LC,LR);
allActiveSetsC=zeros(1,Lm);
allActiveSetsR=zeros(1,Lm); %size can be precomputed but the computational savings are hard to justify
for ij=1:length(C)
    L=nchoosek(C,ij);
    [m,n]=size(L);
    allActiveSetsC=[allActiveSetsC; L zeros(m,Lm-n)]; %#ok<*AGROW>
end
allActiveSetsC;

%LR=length(R);
%allActiveSetsR=zeros(1,LR);
for ij=1:length(R)
    L=nchoosek(R,ij);
    [m,n]=size(L);
    allActiveSetsR=[allActiveSetsR; L zeros(m,Lm-n)];
end
allActiveSetsR;

[numSetsC,~]=size(allActiveSetsC);
[numSetsR,~]=size(allActiveSetsR);

out=[];
for ij=1:numSetsC
    for ik=1:numSetsR
        Ctmp=allActiveSetsC(ij,:);
        Rtmp=allActiveSetsR(ik,:);
        p = perms(Ctmp);
        [m,n] = size(p);
        combMat = zeros(m,2*n);
        combMat(:,1:2:end-1) = repmat(Rtmp,m,1);
        combMat(:,2:2:end) = p;
        out=[out;combMat];
    end
end
out=unique(out,'Rows');

end

