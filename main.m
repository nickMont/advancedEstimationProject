clear;clc;
% Main function for 2D game AI for pop-the-balloon

%
% Game state convention
%        +x
%         |
%         |
%   +y - -
%

tstep=1;
tmax=10;

Game.uType="velstep";
%Options: velstep (step of constant size, from velocity)
%         accel (double integrator)
Game.dt=tstep;

% Red and blue teams, Sred and Sblue
% S contains state information
Sred.quad{1}.state=[-5;2];
Sred.quad{2}.state=[-6;0];
Sred.quad{3}.state=[-5;-2];
Sred.numQuads=3;

Sblue.quad{1}.state=[5;2];
Sblue.quad{2}.state=[6;0];
Sblue.quad{3}.state=[5;-2];
Sblue.numQuads=3;

% Possible controls at each epoch.  -1 is "stationary;" otherwise, apply
% input umax at angle theta.  Computational savings only.
Pred.possibleControls.quad{1}.theta=[-1 0:pi/4:2*pi];
Pred.possibleControls.quad{1}.umax=0.5;
Pred.possibleControls.quad{2}.theta=Pred.possibleControls.quad{1}.theta;
Pred.possibleControls.quad{2}.umax=Pred.possibleControls.quad{1}.umax;
Pred.possibleControls.quad{3}.theta=Pred.possibleControls.quad{1}.theta;
Pred.possibleControls.quad{3}.umax=Pred.possibleControls.quad{1}.umax;

Pblue.possibleControls.quad{1}.theta=[-1 0:pi/4:7*pi/4];
Pblue.possibleControls.quad{1}.umax=0.5;
Pblue.possibleControls.quad{2}.theta=Pblue.possibleControls.quad{1}.theta;
Pblue.possibleControls.quad{2}.umax=Pblue.possibleControls.quad{1}.umax;
Pblue.possibleControls.quad{3}.theta=Pblue.possibleControls.quad{1}.theta;
Pblue.possibleControls.quad{3}.umax=Pblue.possibleControls.quad{1}.umax;


%This can be shunted off to a separate function
n=0;
for ij=1:tstep:tmax
    n=n+1;
    
    % Generate possible controls
    Sred.uFeas=generateGameControls(Pred.possibleControls,Sred.numQuads,2);
    Sblue.uFeas=Sred.uFeas;
    
    % Solve game
    [Cred,Cblue]=generateCostMatrix(Sred,Sblue,Pred,Pblue,Game);
    eq=solveEQ(Cred,Cblue);
    uRed=generateControl(Sred.uFeas,eq,1);
    uBlue=generateControl(Sblue.uFeas,eq,2);
    
    % Propagate to next time step
    
    
    
    
    
    
    
    
end



