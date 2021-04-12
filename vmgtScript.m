%note: this is written as a script rather than as a function to share
%access to workspace variables
SevaTilde=Seva;
SpurTilde=Spur;
gameStateTilde=gameState;
% gameStateTilde.Rtarget.x_target=targetLocation;
% gameStateTilde.xPur=xPur(1:12);
% gameStateTilde.xEva=xPur(13:24);
% gameStateTilde.dt=dt;
% gameStateTilde.kMax=1;
% gameStateTilde.nu=2;
% gameStateTilde.discType='overX';
% gameStateTilde.uMaxP=umax;
% SpurTilde.controlType='vmquad';
% for ik=1:length(uvec)
%     SpurTilde.uMat{ik}=upmax*uvec(ik);
% end
% %SpurTilde.Jname='J_purQuad';
% %SpurTilde.fname='f_dynPurQuad';
% SpurTilde.UseVelMatch=true;
% SevaTilde.controlType='vmquad';
% for ik=1:length(uvec)
%     SevaTilde.uMat{ik}=upmax*uvec(ik);
% end
% %SevaTilde.Jname='J_evaQuad';
% %SevaTilde.fname='f_dynEvaQuad';
% SevaTilde.UseVelMatch=true;
% [up,ue,flag,uPSampled,uESampled,Sminimax,Smisc]=f_dyn2(SpurTilde,SevaTilde,gameStateTilde,zeros(4,1));
% 
% uPurVMGT=uPSampled;
% uEvaVMGT=uESampled;

gameStateTilde.xPur=xPur(1:12);
gameStateTilde.xEva=xPur(13:24);
gameStateTilde.dt=dt;
gameStateTilde.kMax=1;
gameStateTilde.nu=2;
gameStateTilde.discType='overX';
gameStateTilde.uMaxP=umax;
gameStateTilde.uEvaEstForVM=uEvaEst;
SpurTilde.uMat={0}; SevaTilde.uMat={0};
    for ik=1:length(uvec)
        SpurTilde.uMat{ik}=upmax*uvec(ik);
    end
SpurTilde.Jname='J_purQuadTarget';
SpurTilde.fname='f_dynPurQuad';
SpurTilde.UseVelMatch=true;
    for ik=1:length(uvec)
        SevaTilde.uMat{ik}=upmax*uvec(ik);
    end
SpurTilde.Jparams.Q_target=QtargetP;
SpurTilde.Jparams.x_target=targetLocation;
SevaTilde.Jname='J_evaQuadTarget';
SevaTilde.fname='f_dynEvaQuad';
SevaTilde.Jparams.Q_target=QtargetE;
SevaTilde.Jparams.x_target=targetLocation;
gameStateTilde.uEvaEstForVM=uEvaEst;
gameStateTilde.Rtarget.Q_target=QtargetP;
gameStateTilde.Rtarget.x_target=targetLocation;
gameStateTilde.Rtarget.useMotionPrediction=FLAG_tryMotionPredictionInVM2;
gameStateTilde.Rtarget.useNoHeuristics=FLAG_tryVMGTbutBypassHeuristics;
SevaTilde.UseVelMatch=true;
% Propagate to next time step
[up,ue,flag,uPSampled,uESampled,Sminimax,Smisc]=f_dyn2(SpurTilde,SevaTilde,gameStateTilde,zeros(4,1),miscParams);

uPurVMGT=uPSampled;
uEvaVMGT=uESampled;         
                
                
                