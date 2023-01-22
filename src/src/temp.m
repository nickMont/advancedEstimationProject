clear;
beep off;

rngseedno=457;
rng(rngseedno);

discSteps=5:1:15;
nstep=length(discSteps);
tRun_S=cell(nstep,1);
Jp_S=cell(nstep,1);
Je_S=cell(nstep,1);
x_S=cell(nstep,1);

for iter=1:nstep
    
    currentStepsize=discSteps(iter)
    
    Spur.controlType='gt_overx';
    Seva.controlType='gt_overx';
    % gameState_p.controlType='gt_overx';
    
    % load nnTrainSets\nnQuadDyn\network.mat
    % gameState_p.NN=net;
    gameState.tryNN=false;
    
    umax=.5;
    
    dt=0.05;
    t0=0;
    tmax=10;
    
    du=2/(discSteps(iter)-2);
    uvec=-1:du:1;
    utemp=permn(uvec,2)';
    upmax=umax;
    umax=upmax;
    utype.radius=upmax;
    utemp=mapUtempToUvec(utemp,"circle",utype);
    
    n=0;
    z31=zeros(3,1);
    ewxvPur=[z31; z31; 1;0.1;0; z31];
    ewxvEva=[z31; z31; z31; 0;-0.5;0];
    xPur=[ewxvPur;ewxvEva];
    xEva=[ewxvPur;ewxvEva];
    xTrue=xPur;
    
    Qpur=zeros(12,12);
    Qpur(7:9,7:9)=15*eye(3);
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
        JJp=JJp+Jpur;
        JJe=JJe+Jeva;
        
        noise=zeros(24,1);
        xEva=xTrue+noise;
        xPur=xTrue+noise;
        
        xStore=[xStore xTrue];
    end
    tTotal=toc;
    
    tRun_S{iter,1}=tTotal;
    Jp_S{iter,1}=JJp;
    Je_S{iter,1}=JJe;
    x_S{iter,1}=xStore;
    
end
