function [J,Jint,Jf] = evalCostFunc(tHist,stateHist,u1,u2,jNameCont,jNameFinal,params)

n=length(tHist);
jj=zeros(n,1);
for ij=1:n
    x = stateHist(ij,:);
    jj(ij) = feval(jNameCont,x,u1(:,ij),u2(:,ij),params);
end

Jint = trapz(tHist,jj);

[~,ti]=max(tHist);
xF = (stateHist(ti,:))';

Jf = feval(jNameFinal,xF,params);

J = Jint+Jf;

end

