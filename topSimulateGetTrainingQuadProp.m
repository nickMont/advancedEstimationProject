clear;clc;

%rngseedno=457;
%rng(rngseedno)

% Game state convention
%        +x
%         |
%         |
%   +y - -
%

%NOTE: CURRENTLY SET FOR U BASED ON VELOCITY MATCHING SCALE

%load nnvarDynTrained0417.mat
%network1=net;
testNN=false;


u_min=3;
u_max=6;

iter=1000;

uS_S=cell(iter,1);
xk_S=cell(iter,1);
xkp1_S=cell(iter,1);
dt=0.1;



u_range=u_max-u_min;

for iteriter=1:iter
    uS = u_range*rand(4,1)+u_min*ones(4,1);
    e0=rand0(.3,3,1);
    w0=rand0(.1,3,1);
    x0=rand0(20,3,1);
    v0=rand0(2,3,1);
    xk=[e0;w0;x0;v0];
    xkp1=f_dynPurQuad(xk,uS,dt,[]);
    
    uS_S{iteriter,1}=uS;
    xk_S{iteriter,1}=xk;
    xkp1_S{iteriter,1}=xkp1;
end



