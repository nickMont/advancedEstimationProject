clear;clc;loadenv


% [X,T] = simpleseries_dataset;
% net = layrecnet(1:2,10);
% [Xs,Xi,Ai,Ts] = preparets(net,X,T);
% net = train(net,Xs,Ts,Xi,Ai);
% view(net)
% Y = net(Xs,Xi,Ai);
% perf = perform(net,Y,Ts)

load nnTrainSets\nnTrainMMKF_RNN\oneRun.mat
ind=1;
measurements = measStore{ind};
measurements = [xStore{1}(:,1) measurements];
muHist=muStore{ind};
measCell={};
muCell={};
for ij=1:length(muHist)
    measCell{ij}=measurements(:,ij);
    muCell{ij}=muHist(:,ij);
end
T=measCell;
X=muCell;
net=narxnet(1:3,10);
[Xs,Xi,Ai,Ts] = preparets(net,X,{},T);
net = train(net,Xs,Ts,Xi,Ai);
Y = net(Xs,Xi,Ai);
perf = perform(net,Y,Ts)

%https://www.mathworks.com/help/deeplearning/ref/narxnet.html
