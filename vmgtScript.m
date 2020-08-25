%note: this is written as a script rather than as a function to share
%access to workspace variables
SevaTilde=Seva;
SpurTilde=Spur;
gameState.xPur=xPur(1:12);
gameState.xEva=xPur(13:24);
gameState.dt=dt;
gameState.kMax=1;
gameState.nu=2;
gameState.discType='overX';
gameState.uMaxP=umax;
SpurTilde.controlType='vmquad';
for ik=1:length(uvec)
    SpurTilde.uMat{ik}=upmax*uvec(ik);
end

SpurTilde.Jname='J_purQuad';
SpurTilde.fname='f_dynPurQuad';
SpurTilde.UseVelMatch=true;
SevaTilde.controlType='vmquad';
for ik=1:length(uvec)
    SevaTilde.uMat{ik}=upmax*uvec(ik);
end
SevaTilde.Jname='J_evaQuad';
SevaTilde.fname='f_dynEvaQuad';
SevaTilde.UseVelMatch=true;
[up,ue,flag,uPSampled,uESampled,Sminimax,Smisc]=f_dyn2(SpurTilde,SevaTilde,gameState,zeros(4,1));

uPurVMGT=uPSampled;
uEvaVMGT=uESampled;