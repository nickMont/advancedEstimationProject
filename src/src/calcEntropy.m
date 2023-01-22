function H = calcEntropy(P,base)
P = P/sum(P);
H=0;
if nargin==1 %calculate 2bit entropy as default
    for ij=1:length(P)
        H = H - P(ij)*log2(P(ij));
    end
elseif nargin==2
    nn = log(base); %normalization
    for ij=1:length(P)
        H = H - P(ij)*log(P(ij))/nn;
    end
end
end

