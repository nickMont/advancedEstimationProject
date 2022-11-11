function [xkp1,Pkp1] = ukfMeasure(xkbar,Pkbar,zk,Rk,hmeas_name)
alphaUKF=1e-3;
kappaUKF=0;

nz = length(zk);
nx = length(xkbar);

%weights
lambda_u = alphaUKF^2*(kappaUKF + nx + nz) - nx - nz;
c_u = sqrt(nx+nz+lambda_u);
Wm0_u = lambda_u/(nx + nz + lambda_u);
Wmi_u = 1/(2*(nx + nz + lambda_u));
Wc0_u = Wm0_u + 1 - alphaUKF^2 + betaUKF;
Wci_u = Wmi_u;

xBarAugk = [xkbar; zeros(nz,1)];
PBarAugk = blkdiag(Pkbar,Rk);
SxBar = chol(PBarAugk)';

%Push sigma points through hmeas
sp0 = xBarAugk;
spMat = zeros(nx+nz, 2*(nx+nz));  zpMat = zeros(nz,2*(nx+nz));
zp0 = feval(hmeas_name,sp0(1:nx),sp0(nx+1:end),RBIBark,S.rXIMat,mcVeck,P);
for ii=1:2*(nx+nz)
  jj = ii; pm = 1;
  if(ii > (nx + nz)) jj = ii - nx - nz; pm = -1; end
  spMat(:,ii) = sp0 + pm*c_u*SxBar(:,jj);
  zpMat(:,ii) = feval(hmeas_name,spMat(1:nx,ii),spMat(nx+1:end,ii),RBIBark,S.rXIMat,mcVeck,P);
end

%Recombine sigma points
zBark = sum([Wm0_u*zp0, Wmi_u*zpMat],2);
Pzz = Wc0_u*(zp0 - zBark)*(zp0 - zBark)';
Pxz = Wc0_u*(sp0(1:nx) - xBark)*(zp0 - zBark)';
for ii=1:2*(nx+nz)
  Pzz = Pzz + Wci_u*(zpMat(:,ii) - zBark)*(zpMat(:,ii) - zBark)';
  Pxz = Pxz + Wci_u*(spMat(1:nx,ii) - xBark)*(zpMat(:,ii) - zBark)';
end
PzzInv = inv(Pzz);

%Outputs
xkp1 = xBark + Pxz*PzzInv*(zk - zBark);
Pkp1 = PBark - Pxz*PzzInv*Pxz';

end

