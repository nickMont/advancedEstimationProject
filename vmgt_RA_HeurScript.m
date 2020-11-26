%note: this is written as a script rather than as a function to share
%access to workspace variables
SevaTilde=Seva;
SpurTilde=Spur;
gameStateTilde=gameState;
gameStateTilde.xPur=xPur(1:12);
gameStateTilde.xEva=xPur(13:24);
gameStateTilde.dt=dt;
gameStateTilde.kMax=1;
gameStateTilde.nu=2;
gameStateTilde.discType='overX';
gameStateTilde.uMaxP=umax;
SpurTilde.controlType='vmquad';
for ik=1:length(uvec)
    SpurTilde.uMat{ik}=upmax*uvec(ik);
end
SpurTilde.UseVelMatch=true;
SevaTilde.controlType='vmquad';
for ik=1:length(uvec)
    SevaTilde.uMat{ik}=upmax*uvec(ik);
end
SevaTilde.UseVelMatch=true;
if strcmp(heurtype,'both')
    gameStateTilde.Rtarget.useMotionPrediction=true;
    gameStateTilde.Rtarget.useNoHeuristics=false;
elseif strcmp(heurtype,'none')
    gameStateTilde.Rtarget.useMotionPrediction=false;
    gameStateTilde.Rtarget.useNoHeuristics=true;
elseif strcmp(heurtype,'pred_only')
    gameStateTilde.Rtarget.useMotionPrediction=true;
    gameStateTilde.Rtarget.useNoHeuristics=true;
elseif strcmp(heurtype,'heur_only')
    gameStateTilde.Rtarget.useMotionPrediction=false;
    gameStateTilde.Rtarget.useNoHeuristics=false;
else
    error('invalid argument to vmgt_RA_HeurScript');
end
[up,ue,flag,uPSampled,uESampled,Sminimax,Smisc]=f_dyn2(SpurTilde,SevaTilde,gameStateTilde,zeros(4,1));
uPurVMGTH=uPSampled;
uEvaVMGTH=uESampled;