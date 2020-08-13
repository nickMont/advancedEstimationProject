function [xBarkp1,PBarkp1] = ukfPropagate(xk,Pk,uk,Qk,dt,fdyn_name)
[nv,~]=size(Qk);
nx = length(xk);

alphaUKF = 1e-3;
betaUKF = 2;
kappaUKF=0;
lambda_p = alphaUKF^2*(kappaUKF + nx + nv) - nx - nv;
c_p = sqrt(nx+nv+lambda_p);
Wm0_p = lambda_p/(nx + nv + lambda_p);
Wmi_p = 1/(2*(nx + nv + lambda_p));
Wc0_p = Wm0_p + 1 - alphaUKF^2 + betaUKF; Wci_p = Wmi_p;

xHatAugk = [xk; zeros(nv,1)];
PAugk = blkdiag(Pk,Qk);
Sx = chol(PAugk)';
% Assemble sigma points and push these through the dynamics function
sp0 = xHatAugk;
xpMat = zeros(nx,2*(nx+nv));
xp0 = feval(fdyn_name,sp0(1:nx),uk,dt,sp0(nx+1:end));
for ii=1:2*(nx+nv)
    jj = ii; pm = 1;
    if(ii > (nx + nv)) jj = ii - nx - nv; pm = -1; end
    spii = sp0 + pm*c_p*Sx(:,jj);
    xpMat(:,ii) = feval(fdyn_name,spii(1:nx),uk,dt,spii(nx+1:end));
end
% Recombine sigma points
xBarkp1 = sum([Wm0_p*xp0, Wmi_p*xpMat],2);
PBarkp1 = Wc0_p*(xp0 - xBarkp1)*(xp0 - xBarkp1)';
for ii=1:2*(nx+nv)
    PBarkp1 = PBarkp1 + ...
        Wci_p*(xpMat(:,ii) - xBarkp1)*(xpMat(:,ii) - xBarkp1)';
end

end

