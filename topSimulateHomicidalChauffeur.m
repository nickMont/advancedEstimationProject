clear;clc;loadenv;

L = 0.5; %capture range
phi = pi/4; %final capture angle
mu = 0.5; %speed ratio, vmaxE/vmaxP
xBphi = (2*sin(phi)+2*mu*(pi-phi)-L)*sin(phi);
yBphi = -(2*sin(phi)+2*mu*(pi-phi)-L)*cos(phi);

a=1; %a>0
lambdaX_T = -a*sin(phi);
lambdaY_T = -a*cos(phi);
% apprx for consistency with inverse time equations (see Pachter&Coates
%  2018)
lambdaDotX_T = -a*cos(phi);
lambdaDotY_T = a*sin(phi);

Tmax = 2*(pi-phi);

params.mu=mu;
params.phi=phi;
stateT = [xBphi; yBphi; lambdaX_T; lambdaY_T; lambdaDotX_T; lambdaDotY_T];
[tOut,stateOut] = ode45(@(t,x) homicidalChauffeurInverseTime(t,x,params), [Tmax 0], stateT);

x0=stateOut(end,1:2)';
