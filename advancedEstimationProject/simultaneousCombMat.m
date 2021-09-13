function combMat = simultaneousCombMat(C,R)
%assumes that dim(C)==dim(R)
%returns currentMatriR, of size numPairs R dim(C)+dim(R)
%Each row represents a set of pairings, written as
%  C1 R1 C2 R2 C3 R3  etc.

p = perms(C);
[m,n] = size(p);
combMat = zeros(m,2*n);
combMat(:,1:2:end-1) = repmat(R,m,1);
combMat(:,2:2:end) = p;

end

