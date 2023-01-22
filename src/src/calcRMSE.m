clear;clc;

load lumberingPFrun.mat
[nx,~]=size(xPur_part);
PP=zeros(nx,nx);
xmean=zeros(nx,1);

for ij=1:npart
    xmean=xmean+wloc(ij)*xPur_part(:,ij);
end

for ij=1:npart
    dx=xPur_part(:,ij)-xmean;
    PP=PP+wloc(ij)*dx*dx';
end
