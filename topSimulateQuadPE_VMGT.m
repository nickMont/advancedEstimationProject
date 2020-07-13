clear;clc;
beep off;

rngseedno=457;
rng(rngseedno);

%NOTE: TUNING IN VMRGO REQUIRES USING fdynEva AND fdynPur FOR PM SIM
%  SET DRAG PARAMS IN BOTH TO 0.
%NOTE: set the target function to generateCostMatricesVMquad in f_dyn2
%  BEFORE running this; hardcoded the other way to remain backwards
%  compatible


% standard controller:
% runtime: 5.0165e+03
% Jp: 1.0120e+04
% Je: 9.6958e+03

% VMGT:
% runtime: 476.6351
% Jp: 1.0408e+04
% Je: 9.3878e+03

% VMGT w/NN
% runtime: 1.7013e+03
% Jp: 1.0318e+04
% Je: 9.4986e+03

% VMGT BADDIAG
% JJp =
%    1.0016e+04
% JJe =
%    9.0306e+03

% standard BADDIAG
% JJp =
%    1.4131e+04
% JJe =
%    5.9543e+03
   
Spur.controlType='gt_overx';
Seva.controlType='gt_overx';
% gameState_p.controlType='gt_overx';

% load nnTrainSets\nnQuadDyn\network.mat
% gameState_p.NN=net;
gameState.tryNN=false;

umax=.5;

dt=0.1;
t0=0;
tmax=10;

%utemp=permn(-2:0.2:2,2)';
uvec=-1:.2:1;
utemp=permn(uvec,2)';
upmax=umax;
umax=upmax;
utype.radius=upmax;
utemp=mapUtempToUvec(utemp,"circle",utype);

n=0;
z31=zeros(3,1);
ewxvPur=[z31; z31; 1;0.1;0; z31];
ewxvEva=[zeros(12,1)];
xPur=[ewxvPur;ewxvEva];
xEva=[ewxvPur;ewxvEva];
xTrue=xPur;

Qpur=zeros(12,12);
Qpur(7:9,7:9)=diag([5,100,5]);
Qeva=zeros(12,12);
Qeva(7:9,7:9)=5*eye(3);
Rpur=eye(4);
Reva=eye(4);
xStore=xPur;

uPhist=[];
uEhist=[];

JJp=0;
JJe=0;

tic
for t=t0:dt:tmax
    n=n+1;
    
%     upurset=umin:du:umax;
%     %uPur=combvec(upurset,upurset,upurset,upurset);
%     uPur=combvec(upurset,upurset,upurset);
%     uEva=uPur;
    
    % Guessing pursuer
    gameState.xPur=xPur(1:12);
    gameState.xEva=xPur(13:24);
    gameState.dt=dt;
    gameState.kMax=1;
    gameState.nu=2;
    gameState.discType='overX';
    gameState.uMaxP=umax;
    if strcmp(Spur.controlType,'vmquad')
        for ik=1:length(uvec)
            Spur.uMat{ik}=upmax*uvec(ik);
        end
    end
    if strcmp(Spur.controlType,'gt_overx')
        for ik=1:length(utemp)
            Spur.uMat{ik}=utemp(:,ik);
        end
    end
    Spur.Jname='J_purQuad';
    Spur.fname='f_dynPurQuad';
    Spur.Jparams.Q=Qpur;
    Spur.Jparams.Rself=Rpur;
    Spur.Jparams.Ropp=zeros(4,4);
    Spur.UseVelMatch=true;
    if strcmp(Seva.controlType,'vmquad')
        for ik=1:length(uvec)
            Seva.uMat{ik}=upmax*uvec(ik);
        end
    end
    if strcmp(Seva.controlType,'gt_overx')
        for ik=1:length(utemp)
            Seva.uMat{ik}=utemp(:,ik);
        end
    end
    Seva.Jname='J_evaQuad';
    Seva.fname='f_dynEvaQuad';
    Seva.Jparams.Q=Qeva;
    Seva.Jparams.Rself=Reva;
    Seva.Jparams.Ropp=zeros(4,4);
    Seva.UseVelMatch=true;
    % Propagate to next time step
    [up,ue,flag,uPSampled,uESampled]=f_dyn2(Spur,Seva,gameState,zeros(4,1));
    uPurTrue=uPSampled;
    uEvaTrue=uESampled;
    
    uPhist=[uPhist uPurTrue];
    uEhist=[uEhist uEvaTrue];
    xTrue(1:12)=f_dynPurQuad(xTrue(1:12),uPurTrue(:,1),dt,zeros(2,1));
    xTrue(13:24)=f_dynEvaQuad(xTrue(13:24),uEvaTrue(:,1),dt,zeros(2,1));
    xD=[zeros(24,1) xTrue];
    
    Jpur=feval(Spur.Jname,xD(1:12,:),xD(13:24,:),uPurTrue,uEvaTrue,Spur.Jparams);
    Jeva=feval(Seva.Jname,xD(13:24,:),xD(1:12,:),uEvaTrue,uPurTrue,Seva.Jparams);
    JJp=JJp+Jpur
    JJe=JJe+Jeva
    
    noise=zeros(24,1);
    xEva=xTrue+noise;
    xPur=xTrue+noise;
    
    xStore=[xStore xTrue];
end
tTotal=toc

%     load quaddat.mat;
indsamp=1:5:100;
xP2d=xStore(7:8,indsamp);
xE2d=xStore(19:20,indsamp);
% mx=(xP2d(2,1)-xE2d(2,1))/(xP2d(1,1)-xE2d(1,1));
% xPlin_x=-5:0.3:1;
% xPlin_y=xPlin_x*mx;
% xElin_x=-6:0.3:0;
% xElin_y=xElin_x*mx;

% figure(1);clf;
% figset
% plot(xP2d(1,:),xP2d(2,:),'-*b');
% hold on
% plot(xE2d(1,:),xE2d(2,:),'-Or');
% % hold on
% % plot(xPlinCompare(1,:),xPlinCompare(2,:),'-.*k');
% % hold on
% % plot(xElinCompare(1,:),xElinCompare(2,:),'-.og');
% axis([-6 2 -6 2])
% xlabel('x-position (m)')
% ylabel('y-position (m)')
% legend('Pursuer trajectory, quad dynamics','Evader trajectory, quad dynamics','Pursuer trajectory, point mass','Evader trajectory, point mass');
% figset


