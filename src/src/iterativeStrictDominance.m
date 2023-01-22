function [VrowPrune,VcolPrune,rowsetPrune,colsetPrune] = iterativeStrictDominance(Vrow,Vcol,maxiter,Vr2,Vc2)
%note: ISD takes VALUE not COST
%Vr2/Vc2 are upper values
[r,c]=size(Vrow);
rowsetPrune=1:r;
colsetPrune=1:c;

if nargin==2
    maxiter=100;
end
if nargin<=3
    Vr2=Vrow;
    Vc2=Vcol;
end

VrowPrune=Vrow;
VcolPrune=Vcol;

contvar=true;
iter=0;
while contvar
    contvar=false;
    
    [R,~]=size(VrowPrune);
    [~,C]=size(VcolPrune);
    [VrowPrune,ind]=strictDominance(VrowPrune,'row',Vr2(rowsetPrune,colsetPrune));
    rowsetPrune=rowsetPrune(ind);
    VcolPrune=VcolPrune(ind,:);
    [VcolPrune,ind]=strictDominance(VcolPrune,'col',Vc2(rowsetPrune,colsetPrune));
    colsetPrune=colsetPrune(ind);
    VrowPrune=VrowPrune(:,ind);
    
    iter=iter+1;
    [R2,~]=size(VrowPrune);
    [~,C2]=size(VcolPrune);
    if ((R~=R2 || C~=C2) && iter<maxiter)
        contvar=true;
    end
end


end

