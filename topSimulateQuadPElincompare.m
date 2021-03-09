clear;clc;

umin=-0.6;
umax=0.6;
du=0.2;
du2=100;

dt=0.1;
t0=0;
tmax=10;

n=0;
z31=zeros(3,1);
ewxvPur=[z31; z31; 1;0.1;0; z31];
ewxvEva=[zeros(12,1)];
thetaOffset=atan2(ewxvPur(8),ewxvPur(7));
xPur=[sqrt(.1^1+1);0; ewxvPur(10:11); 0;0; ewxvEva(10:11)];
xEva=[ewxvPur;ewxvEva];
xTrue=xPur;

%control values matched such that 0.05m in quad is 0.05m in lin
Qpur=zeros(4,4);
Qpur(1:2,1:2)=100*eye(2);
Qeva=zeros(4,4);
Qeva(1:2,1:2)=100*eye(2);
Rpur=2*eye(2);
Reva=2*eye(2);
xStore=xPur;

uPhist=[];
uEhist=[];

uvec=-1:0.1:1;
utemp=permn(uvec,2)';
upmax=2;
umax=upmax;
utype.radius=upmax;
utemp=mapUtempToUvec(utemp,"circle",utype);

tic
for t=t0:dt:tmax
    n=n+1;
    
    upurset=umin:du:umax;
    %uPur=combvec(upurset,upurset,upurset,upurset);
    uPur=combvec(upurset,upurset);
    uEva=uPur;
    
    % Guessing pursuer
    gameState_p.xPur=xPur(1:4);
    gameState_p.xEva=xPur(5:8);
    gameState_p.dt=dt;
    gameState_p.kMax=1;
    gameState_p.nu=2;
    gameState_p.discType='overX';
    Spur_p.uMat={0}; Seva_p.uMat={0};
    for ik=1:length(uPur)
        Spur_p.uMat{ik}=utemp(:,ik);
    end
    Spur_p.Jname='J_pur';
    Spur_p.fname='f_dyn0D';
    Spur_p.Jparams.Q=Qpur;
    Spur_p.Jparams.Rself=Rpur;
    Spur_p.Jparams.Ropp=zeros(2,2);
    for ik=1:length(uEva)
        Seva_p.uMat{ik}=utemp(:,ik);
    end
    Seva_p.Jname='J_eva';
    Seva_p.fname='f_dyn0D';
    Seva_p.Jparams.Q=Qeva;
    Seva_p.Jparams.Rself=Reva;
    Seva_p.Jparams.Ropp=zeros(2,2);
    % Propagate to next time step
    [up,ue,flag,uPSampled,uESampled]=f_dyn(Spur_p,Seva_p,gameState_p,zeros(4,1))
    flagflag=flag
    if flag==0
        upp=randsample(1:length(Spur_p.uMat),1,true,up);
        uPurTrue=Spur_p.uMat{upp}(:,1);
        uEvaTrue=zeros(gameState_p.nu,gameState_p.kMax);
        for ik=1:length(Seva_p.uMat)
            uEvaTrue=uEvaTrue+ue(ik)*Seva_p.uMat{ik}(:,1);
        end
        QinflatePur=Qnoisepur+blkdiag(zer2,1*eye2);
        %scaleS{n}(1)=uvec(upp);
    else
        uPurTrue=up;
        uEvaTrue=ue;
        %scaleS{n}(1)=uvec(tmp1);
    end
    

    
%     if strcmp(gameState_p.discType,'overX')
%         uPurTrue=uPSampled;
%         uEvaTrue=uESampled;
%         %         des=xTrue(1:12);
%         %         des(7:9)=des(7:9)+uPurTrue;
%         %         uPurTrue = quadController(xTrue(1:12),zeros(4,1),zeros(3,1),des,1,zeros(12,1));
%         %         des=xTrue(13:24);
%         %         des(7:9)=des(7:9)+uEvaTrue;
%         %         uEvaTrue = quadController(xTrue(13:24),zeros(4,1),zeros(3,1),des,1,zeros(12,1));
%     end
    uPhist=[uPhist uPurTrue];
    uEhist=[uEhist uEvaTrue];
    f_dyn0D(xTrue(1:4),uPurTrue(:,1),dt,zeros(2,1))
    xTrue(1:4)=f_dyn0D(xTrue(1:4),uPurTrue(:,1),dt,zeros(2,1));
    xTrue(5:8)=f_dyn0D(xTrue(5:8),uEvaTrue(:,1),dt,zeros(2,1));
    xPur=xTrue;
    
%     noise=zeros(24,1);
%     xEva=xTrue+noise;
%     xPur=xTrue+noise;
%     
    xStore=[xStore xTrue]
end
tTotal=toc

%     load quaddat.mat;
indsamp=1:5:100;
xP2d=xStore(1:2,indsamp);
xE2d=xStore(5:6,indsamp);
R=[cos(thetaOffset) -sin(thetaOffset);sin(thetaOffset) cos(thetaOffset)];
mx=(xP2d(2,1)-xE2d(2,1))/(xP2d(1,1)-xE2d(1,1));
xPlin_x=-5:0.3:1;
xPlin_y=xPlin_x*mx;
xElin_x=-6:0.3:0;
xElin_y=xElin_x*mx;
xP2d=R*xP2d;
xE2d=R*xE2d;
xPlinCompare=xP2d;
xElinCompare=xE2d;

figure(1);clf;
figset
plot(xP2d(1,:),xP2d(2,:),'-*b');
hold on
plot(xE2d(1,:),xE2d(2,:),'-Or');
% hold on
% plot(xPlin_x,xPlin_y,'-.*k');
% hold on
% plot(xElin_x,xElin_y,'-.og');
axis([-6 2 -6 2])
xlabel('x-position (m)')
ylabel('y-position (m)')
legend('Pursuer trajectory, quad dynamics','Evader trajectory, quad dynamics','Pursuer trajectory, point mass','Evader trajectory, point mass');
figset


