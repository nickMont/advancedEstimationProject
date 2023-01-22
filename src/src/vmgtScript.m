%note: this is written as a script rather than as a function to share
%access to workspace variables
SevaTilde=Seva;
SpurTilde=Spur;
gameStateTilde=gameState;

% Guessing pursuer
gameStateTilde.xPur=xPur(1:12);
gameStateTilde.xEva=xPur(13:24);
gameStateTilde.dt=dt;
gameStateTilde.kMax=1;
gameStateTilde.nu=2;
gameStateTilde.discType='overX';
gameStateTilde.uMaxP=upmax;
gameStateTilde.uEvaEstForVM=uEvaEst;
SpurTilde.uMat={}; SevaTilde.uMat={};
    for ik=1:length(uvec)
        SpurTilde.uMat{ik}=upmax*uvec(ik);
    end
SpurTilde.Jname='J_purQuadTarget';
SpurTilde.fname='f_dynPurQuad';
SpurTilde.UseVelMatch=true;
    for ik=1:length(uvec)
        SevaTilde.uMat{ik}=uemax*uvec(ik);
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
[upT,ueT,flagT,uPSampledT,uESampledT,SminimaxT,SmiscT,Smmkf2]=f_dyn2(SpurTilde,SevaTilde,gameStateTilde,zeros(4,1),miscParams);
% uPurTrueStr{iT}=uPSampled;
% uEvaTrueStr{iT}=uESampled;
% uEvaNash=uESampled;
% dewxv=f_dynEvaQuad(xTrue(13:24),uEvaNash,dt,zeros(2,1))-gameStateTilde.xEva;
% dx=dewxv(7:8); dx=unit_vector(dx);
% uEvaEst=umax*dx;
% [up,ue,flag,uPSampled,uESampled,Sminimax,Smisc]=f_dyn2(SpurTilde,SevaTilde,gameStateTilde,zeros(4,1),miscParams);

uPurVMGT=uPSampledT;
uEvaVMGT=uESampledT;

if exist('flagUseMeanInsteadOfLH2','var')
    if flagUseMeanInsteadOfLH2
        uPurVMGT=Smmkf2.uValP;
        uEvaVMGT=Smmkf2.uValE;
        Ru2InflateP=Smmkf2.RadditiveInflateP;
        Ru2InflateE=Smmkf2.RadditiveInflateE;
    end
end



                
                
                