function C = comb2(v1,m1)
% Performs the operation of combvec. v1 must be a vector, m1 can be a
%  vector or matrix.
%Note: ONLY HANDLES COLUMNS

a=length(v1);
[b,c]=size(m1);
C=zeros(a*b,c+1);
na=0;
for ij=1:a
    nb=0;
    for ik=1:b
        na=na+1;
        C(na,:)=[v1(ij) m1(ik,:)];
    end
end

end

