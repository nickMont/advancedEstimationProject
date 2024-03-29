clear;clc;

umin=-0.6;
umax=0.6;
du=0.3;
du2=100;

dt=0.1;
t0=0;
tmax=10;

n=0;
z31=zeros(3,1);
ewxvPur=[z31; z31; 1;0.1;0; z31];
ewxvEva=[zeros(12,1)];
xPur=[ewxvPur;ewxvEva];
xEva=[ewxvPur;ewxvEva];
xTrue=xPur;

Qpur=zeros(12,12);
Qpur(7:9,7:9)=5*eye(3);
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
    
    upurset=umin:du:umax;
    %uPur=combvec(upurset,upurset,upurset,upurset);
    uPur=combvec(upurset,upurset,upurset);
    uEva=uPur;
    
    % Guessing pursuer
    gameState_p.xPur=xPur(1:12);
    gameState_p.xEva=xPur(13:24);
    gameState_p.dt=dt;
    gameState_p.kMax=1;
    gameState_p.nu=2;
    gameState_p.discType='overX';
    Spur_p.uMat={0}; Seva_p.uMat={0};
    for ik=1:length(uPur)
        Spur_p.uMat{ik}=uPur(:,ik);
    end
    Spur_p.Jname='J_purQuad';
    Spur_p.fname='f_dynPurQuad';
    Spur_p.Jparams.Q=Qpur;
    Spur_p.Jparams.Rself=Rpur;
    Spur_p.Jparams.Ropp=zeros(4,4);
    for ik=1:length(uEva)
        Seva_p.uMat{ik}=uEva(:,ik);
    end
    Seva_p.Jname='J_evaQuad';
    Seva_p.fname='f_dynEvaQuad';
    Seva_p.Jparams.Q=Qeva;
    Seva_p.Jparams.Rself=Reva;
    Seva_p.Jparams.Ropp=zeros(4,4);
    % Propagate to next time step
    [up,ue,flag,uPSampled,uESampled]=f_dyn2(Spur_p,Seva_p,gameState_p,zeros(4,1));
    uPurTrue=up;
    uEvaTrue=ue;
    
    if strcmp(gameState_p.discType,'overX')
        uPurTrue=uPSampled;
        uEvaTrue=uESampled;
        %         des=xTrue(1:12);
        %         des(7:9)=des(7:9)+uPurTrue;
        %         uPurTrue = quadController(xTrue(1:12),zeros(4,1),zeros(3,1),des,1,zeros(12,1));
        %         des=xTrue(13:24);
        %         des(7:9)=des(7:9)+uEvaTrue;
        %         uEvaTrue = quadController(xTrue(13:24),zeros(4,1),zeros(3,1),des,1,zeros(12,1));
    end
    uPhist=[uPhist uPurTrue];
    uEhist=[uEhist uEvaTrue];
    xTrue(1:12)=f_dynPurQuad(xTrue(1:12),uPurTrue(:,1),dt,zeros(2,1));
    xTrue(13:24)=f_dynEvaQuad(xTrue(13:24),uEvaTrue(:,1),dt,zeros(2,1));
    
    noise=zeros(24,1);
    xEva=xTrue+noise;
    xPur=xTrue+noise;
    
    xD=[zeros(24,1) xTrue];
    Jpur=feval(Spur_p.Jname,xD(1:12,:),xD(13:24,:),uPurTrue,uEvaTrue,Spur_p.Jparams);
    Jeva=feval(Seva_p.Jname,xD(13:24,:),xD(1:12,:),uEvaTrue,uPurTrue,Seva_p.Jparams);
    JJp=JJp+Jpur
    JJe=JJe+Jeva
    
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

figure(1);clf;
figset
plot(xP2d(1,:),xP2d(2,:),'-*b');
hold on
plot(xE2d(1,:),xE2d(2,:),'-Or');
% hold on
% plot(xPlinCompare(1,:),xPlinCompare(2,:),'-.*k');
% hold on
% plot(xElinCompare(1,:),xElinCompare(2,:),'-.og');
axis([-6 2 -6 2])
xlabel('x-position (m)')
ylabel('y-position (m)')
legend('Pursuer trajectory, quad dynamics','Evader trajectory, quad dynamics','Pursuer trajectory, point mass','Evader trajectory, point mass');
figset


