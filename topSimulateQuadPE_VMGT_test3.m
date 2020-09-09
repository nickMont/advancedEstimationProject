% clear;clc;
beep off;

rngseedno=457;
rng(rngseedno);

%NOTE: TUNING IN VMRGO REQUIRES USING fdynEva AND fdynPur FOR PM SIM
%  SET DRAG PARAMS IN BOTH TO 0.
%NOTE: set the target function to generateCostMatricesVMquad in f_dyn2
%  BEFORE running this; hardcoded the other way to remain backwards
%  compatible

Jp_S=zeros(3,1);
Je_S=zeros(3,1);
tT_S=zeros(3,1);
uP_S=zeros(4,2,3);
uE_S=zeros(4,2,3);

for iTemp=1:3

clearvars -except Jp_S Je_S tT_S uP_S uE_S iTemp

if iTemp==1
    useGameTheoreticController=true;
    usePureVelMatchController=false;
    Spur.controlType='gt_overx';
    Seva.controlType=Spur.controlType;
%     continue
elseif iTemp==2
    useGameTheoreticController=true;
    usePureVelMatchController=false;
    Spur.controlType='vmquad';
    Seva.controlType=Spur.controlType;
%     continue
elseif iTemp==3
    useGameTheoreticController=false;
    usePureVelMatchController=true;
%     continue
end

scaleVec=1.5; %VM; magnitude of desired uE control relative to uP control
vmtune=.2; %VM; deceleration parameter

% load nnTrainSets\nnQuadDyn\network.mat
% gameState_p.NN=net;
gameState.tryNN=false;

umax=1;

dt=1/10;
t0=0;
tmax=1;

%utemp=permn(-2:0.2:2,2)';
uvec=-1:2/14:1;
utemp=permn(uvec,2)';
upmax=umax;
uemax=upmax;
utype.radius=upmax;
utemp=mapUtempToUvec(utemp,"circle",utype);

n=0;
z31=zeros(3,1);
ewxvEva=[z31; z31; 1;0;0; 0;.1;0];
ewxvPur=[zeros(12,1)];
xPur=[ewxvPur;ewxvEva];
xEva=[ewxvPur;ewxvEva];
xTrue=xPur;

Qpur=zeros(12,12);
Qpur(7:9,7:9)=diag([5 5 0]);
Qpur(10:12,10:12)=5*eye(3);
Qeva=zeros(12,12);
Qeva(7:9,7:9)=diag([20 20 20]);
Qeva(10:12,10:12)=5*eye(3);
Rpur=5*eye(4);
Reva=5*eye(4);
xStore=xPur;

uPhist=[];
uEhist=[];

JJp=0;
JJe=0;

xhatE=ewxvEva;
PhatE=0.001*eye(length(ewxvEva));
Rk=0.1*eye(length(ewxvEva));

tic
for t=t0:dt:tmax
    t_current = t
    n=n+1;
    
    %     upurset=umin:du:umax;
    %     %uPur=combvec(upurset,upurset,upurset,upurset);
    %     uPur=combvec(upurset,upurset,upurset);
    %     uEva=uPur;
    Spur.Jname='J_purQuad';
    Seva.Jname='J_evaQuad';
    Spur.Jparams.Q=Qpur;
    Spur.Jparams.Rself=Rpur;
    Spur.Jparams.Ropp=zeros(4,4);
    Seva.Jparams.Q=Qeva;
    Seva.Jparams.Rself=Reva;
    Seva.Jparams.Ropp=zeros(4,4);
    xp=[xTrue(7:8);xTrue(10:11)];xe=[xTrue(19:20);xTrue(22:23)];
    uP=vmRGVO_tune(xp,xe,umax,2,dt,zeros(2,1),vmtune);
    uE=-scaleVec*uP;
    xdP=f_dynPur(xp,uP,dt,zeros(2,1));
    xdE=f_dynEva(xp,uP,dt,zeros(2,1));
    
    if useGameTheoreticController
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
                Spur.uMat{ik}=utemp(:,ik)*dt*upmax;
            end
            Spur.uMat{ik+1}=xdP(1:2);
        end
        Spur.Jname='J_purQuad';
        Spur.fname='f_dynPurQuad';
        Spur.UseVelMatch=true;
        if strcmp(Seva.controlType,'vmquad')
            for ik=1:length(uvec)
                Seva.uMat{ik}=upmax*uvec(ik);
            end
        end
        if strcmp(Seva.controlType,'gt_overx')
            for ik=1:length(utemp)
                Seva.uMat{ik}=utemp(:,ik)*dt*uemax;
            end
            Seva.uMat{ik+1}=xdE(1:2);
        end
        Seva.Jname='J_evaQuad';
        Seva.fname='f_dynEvaQuad';
        Seva.UseVelMatch=true;
        % Propagate to next time step
        [up,ue,flag,uPSampled,uESampled]=f_dyn2(Spur,Seva,gameState,zeros(4,1));
        uPurTrue=uPSampled;
        uEvaTrue=uESampled;
    elseif usePureVelMatchController
        xp=[xTrue(7:8);xTrue(10:11)];xe=[xTrue(19:20);xTrue(22:23)];
        uP=vmRGVO_tune(xp,xe,umax,2,dt,zeros(2,1),vmtune);
        uE=-scaleVec*uP;
        if norm(uE)>umax
            uE=umax*uE/norm(uE);
        end
        xd = xPur(13:24); xd(9)=0;
        uEvaTrue = quadControllerACCONLY(xd, zeros(4,1), 3, [uE;0],0);
        xd = xPur(1:12); xd(9)=0;
        uPurTrue = quadControllerACCONLY(xd, zeros(4,1), 3, [uP;0],0);
    else
        error('No controller specified')
    end
    
    uPhist=[uPhist uPurTrue];
    uEhist=[uEhist uEvaTrue];
    xTrue(1:12)=f_dynPurQuad(xTrue(1:12),uPurTrue(:,1),dt,zeros(2,1));
    xTrue(13:24)=f_dynEvaQuad(xTrue(13:24),uEvaTrue(:,1),dt,zeros(2,1));
    xD=[zeros(24,1) xTrue];
    
    Jpur=feval(Spur.Jname,xD(1:12,:),xD(13:24,:),uPurTrue,uEvaTrue,Spur.Jparams);
    Jeva=feval(Seva.Jname,xD(13:24,:),xD(1:12,:),uEvaTrue,uPurTrue,Seva.Jparams);
    JJp=JJp+Jpur
    JJe=JJe+Jeva
    
    [xhatEp1,Pp1]=ukfPropagate(xhatE,PhatE,uEvaTrue,[],dt,'f_dynEvaQuad');
    
    zMeas=xTrue(13:24)+chol(Rk)*rand(12,1);
    [xhatE,PhatE]=kfMeasure(xhatEp1,Pp1,zMeas,eye(length(ewxvEva)),Rk);
    
%     xhatE-xTrue(13:24)
    
    noise=zeros(24,1);
    xEva=xTrue+noise;
    xPur=xTrue+noise;
    
    uP_S(:,n,iTemp)=uPurTrue;
    uE_S(:,n,iTemp)=uEvaTrue;
    
    xStore=[xStore xTrue];
end
tTotal=toc;
Jp_S(iTemp)=JJp;
Je_S(iTemp)=JJe;
tT_S(iTemp)=tTotal;

% %     load quaddat.mat;
indsamp=1:1:size(xStore,2);
xP2d=xStore(7:8,indsamp);
xE2d=xStore(19:20,indsamp);
% % mx=(xP2d(2,1)-xE2d(2,1))/(xP2d(1,1)-xE2d(1,1));
% % xPlin_x=-5:0.3:1;
% % xPlin_y=xPlin_x*mx;
% % xElin_x=-6:0.3:0;
% % xElin_y=xElin_x*mx;
%
if iTemp==1
figure(1);clf;
figset
plot(xP2d(1,:),xP2d(2,:),'-*k');
hold on
plot(xE2d(1,:),xE2d(2,:),'-Ok');
axis([-1 2 -1 2])
xlabel('x-position (m)')
ylabel('y-position (m)')
legend('Pursuer, hybrid','Evader, hybrid');
figset
end

end


fprintf('GT / VMGT / VM \n')
for iT=1:3
    fprintf('%0.4d     %0.4d     %0.4d \n',Jp_S(iT,1),Je_S(iT,1),tT_S(iT,1))
end


