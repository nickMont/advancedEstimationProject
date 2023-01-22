function [mm]=minimax1(A)
% Column player is trying to maximize
% Row player is trying to minimize

[~,b]=size(A);
mvalvec=zeros(b,1);
mdexvec=zeros(b,1);
for ij=1:b
    col=A(:,ij);
    [mval,mindex]=min(col);
    mdexvec(ij)=mindex;
    mvalvec(ij)=mval;
end

[mx,mi] = max(mvalvec); %#ok<ASGLU>

mm = [mdexvec(mi) mi];

end