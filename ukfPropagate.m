function [xBarkp1,PBarkp11] = ukfPropagate(xk,Pk,uk,Qk,fdyn_name)
[nv,~]=size(Qk);
xHatAugk = [xk; zeros(nv,1)];
PAugk = blkdiag(Pk,Qk);
Sx = chol(PAugk)';
% Assemble sigma points and push these through the dynamics function
sp0 = xHatAugk;
xpMat = zeros(nx,2*(nx+nv));
xp0 = f_dynamics(sp0(1:nx),uk,sp0(nx+1:end),S.delt,RBIHatk,P);
for ii=1:2*(nx+nv)
  jj = ii; pm = 1;
  if(ii > (nx + nv)) jj = ii - nx - nv; pm = -1; end
  spii = sp0 + pm*c_p*Sx(:,jj);
  xpMat(:,ii) = feval(fdyn_name,spii(1:nx),uk,spii(nx+1:end),S.delt);
end
% Recombine sigma points
xBarkp1 = sum([Wm0_p*xp0, Wmi_p*xpMat],2);
PBarkp1 = Wc0_p*(xp0 - xBarkp1)*(xp0 - xBarkp1)';
for ii=1:2*(nx+nv)
  PBarkp1 = PBarkp1 + ...
            Wci_p*(xpMat(:,ii) - xBarkp1)*(xpMat(:,ii) - xBarkp1)';
end
% ekp1 = xBarkp1(7:9);
% RBIBarkp1 = euler2dcm(ekp1)*RBIHatk;
% xBarkp1(7:9) = zeros(3,1);
% % Set k = kp1 in preparation for next iteration
% RBIBark = RBIBarkp1; xBark = xBarkp1; PBark = PBarkp1;
end

