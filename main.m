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

Qpur = diag([100 100 0 0]); Rpur = diag([0.1 1]);
Qeva = diag([10 10 0 0]); Reva = diag([1 1]);

%This can be shunted off to a separate function
n=0;
purState=[[0 0 1 1]'; [4 4 0 0]'];
for ij=1:tstep:tmax
    n=n+1;
    
    % Generate possible controls
    gameState_p.xPur=purState(1:4);
    gameState_p.xEva=purState(5:8);
    gameState_p.dt=tstep;
    gameState_p.kMax=1;
    gameState_p.nu=2;
    utemp=permn(-2:.5:1,2)';
    for ik=1:length(utemp)
        Spur_p.uMat{ik}=utemp(:,ik);
    end
    Spur_p.Jname='J_pur';
    Spur_p.fname='f_dynPur';
    Spur_p.Jparams.Q=Qpur;
    Spur_p.Jparams.Rself=Rpur;
    Spur_p.Jparams.Ropp=zeros(2,2);
    Seva_p.uMat = Spur_p.uMat;
    Seva_p.Jname='J_eva';
    Seva_p.fname='f_dynEva';
    Seva_p.Jparams.Q=Qeva;
    Seva_p.Jparams.Rself=Reva;
    Seva_p.Jparams.Ropp=zeros(2,2);
    % Propagate to next time step
    
    purState=f_dyn(Spur_p,Seva_p,gameState_p,zeros(4,1));
    pe=purState(1:8);    
    
    
end



