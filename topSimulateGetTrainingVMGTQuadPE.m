clear;clc;
beep off;

%OLD DO NOT USE

% Spur_p.controlType='gt_overx';
Seva_p.controlType='vmquad';
% gameState_p.controlType='gt_overx';

nmax=1000;

load nnTrainSets\nnQuadDyn\network.mat
gameState_p.tryNN=false;
gameState_p.NN=net;

umax=0.1;

dt=0.1;
t0=0;
tmax=10;

%utemp=permn(-2:0.2:2,2)';
uvec=-1:.1:1;
utemp=permn(uvec,2)';
upmax=umax;
umax=upmax;
utype.radius=upmax;
utemp=mapUtempToUvec(utemp,"circle",utype);

tic

ewxv_S=cell(nmax,1);
uPuE_S=cell(nmax,1);

for iter=1:nmax
    
    z31=zeros(3,1);
    e0=rand0(.3,3,1);
    w0=rand0(.1,3,1);
    x0=rand0(20,3,1);
    v0=rand0(2,3,1);
    xk=[e0;w0;x0;v0];
    ewxvPur=xk;
    e0=rand0(.3,3,1);
    w0=rand0(.1,3,1);
    x_delta=rand0(3,3,1);
    v0=rand0(2,3,1);
    xk=[e0;w0;x0+x_delta;v0];
    ewxvEva=xk;
    xPur=[ewxvPur;ewxvEva];
    xEva=[ewxvPur;ewxvEva];
    xTrue=xPur;
    
    Qpur=zeros(12,12);
    Qpur(7:9,7:9)=100*diag(rand(3,1));
    Qeva=zeros(12,12);
    Qeva(7:9,7:9)=100*diag(rand(3,1));
    Rpur=10*diag(rand(4,1));
    Reva=10*diag(rand(4,1));
    xStore=xPur;
    
    uPhist=[];
    uEhist=[];
    
    JJp=0;
    JJe=0;
    
    % Guessing pursuer
    gameState_p.xPur=xPur(1:12);
    gameState_p.xEva=xPur(13:24);
    gameState_p.dt=dt;
    gameState_p.kMax=1;
    gameState_p.nu=2;
    gameState_p.discType='overX';
    gameState_p.uMaxP=umax;
    if strcmp(Spur_p.controlType,'vmquad')
        for ik=1:length(uvec)
            Spur_p.uMat{ik}=upmax*uvec(ik);
        end
    end
    if strcmp(Spur_p.controlType,'gt_overx')
        for ik=1:length(utemp)
            Spur_p.uMat{ik}=utemp(:,ik);
        end
    end
    Spur_p.Jname='J_purQuad';
    Spur_p.fname='f_dynPurQuad';
    Spur_p.Jparams.Q=Qpur;
    Spur_p.Jparams.Rself=Rpur;
    Spur_p.Jparams.Ropp=zeros(4,4);
    Spur_p.UseVelMatch=true;
    if strcmp(Seva_p.controlType,'vmquad')
        for ik=1:length(uvec)
            Seva_p.uMat{ik}=upmax*uvec(ik);
        end
    end
    if strcmp(Seva_p.controlType,'gt_overx')
        for ik=1:length(utemp)
            Seva_p.uMat{ik}=utemp(:,ik);
        end
    end
    Seva_p.Jname='J_evaQuad';
    Seva_p.fname='f_dynEvaQuad';
    Seva_p.Jparams.Q=Qeva;
    Seva_p.Jparams.Rself=Reva;
    Seva_p.Jparams.Ropp=zeros(4,4);
    Seva_p.UseVelMatch=true;
    % Propagate to next time step
    [up,ue,flag,uPSampled,uESampled]=f_dyn2(Spur_p,Seva_p,gameState_p,zeros(4,1));
    uPurTrue=uPSampled;
    uEvaTrue=uESampled;
    
    uPuE_S{iter,1}=[uPurTrue;uEvaTrue];
    
%     uPhist=[uPhist uPurTrue];
%     uEhist=[uEhist uEvaTrue];
%     xTrue(1:12)=f_dynPurQuad(xTrue(1:12),uPurTrue(:,1),dt,zeros(2,1));
%     xTrue(13:24)=f_dynEvaQuad(xTrue(13:24),uEvaTrue(:,1),dt,zeros(2,1));
%     xD=[zeros(24,1) xTrue];
%     
%     Jpur=feval(Spur_p.Jname,xD(1:12,:),xD(13:24,:),uPurTrue,uEvaTrue,Spur_p.Jparams);
%     Jeva=feval(Seva_p.Jname,xD(13:24,:),xD(1:12,:),uEvaTrue,uPurTrue,Seva_p.Jparams);
%     JJp=JJp+Jpur
%     JJe=JJe+Jeva
%     
%     noise=zeros(24,1);
%     xEva=xTrue+noise;
%     xPur=xTrue+noise;
%     
%     xStore=[xStore xTrue];
end

tTotal=toc



