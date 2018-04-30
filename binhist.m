function [binX,binW] = binhist(dd)
binX=0;
binX=[binX 10.^(0:.25:5)];
binW=zeros(length(binX),1);

for ij=1:length(dd)
    if dd(ij)<1e-9
        binW(1)=binW(1)+1;
    else
        L10=log10(dd(ij));
        ind=find(binX>=L10,1);
        binW(ind)=binW(ind)+1;
    end
end
binW=binW/length(dd);


end

