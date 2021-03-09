function xyDot = homicidalChauffeurForwardFOH(t,xy,params)
%assumes first-order-hold on Psi and u

PsiMat = params.PsiMat;
uPMat = params.uPMat;
% Find first index after control switches
indU = find(uPMat(:,1)>t,1);
u = uPMat(indU-1,2);
Psi = 0;
if strcmp(params.PsiType,'step')
    indP = find(PsiMat(:,1)>t,1);
    Psi = PsiMat(indP-1,2);
elseif strcmp(params.PsiType,'linear')
    Psi = PsiMat(1)*t+PsiMat(2);
elseif strcmp(params.PsiType,'interp')
    Psi = interp1(PsiMat(:,1),PsiMat(:,2),t);
end

mu = params.mu;
x = xy(1);
y = xy(2);

xDot = -y*u+mu*sin(Psi);
yDot = x*u-1+mu*cos(Psi);

xyDot=[xDot;yDot];

end

