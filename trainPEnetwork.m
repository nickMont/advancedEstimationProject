clear;clc;

%Trains for constant Q,R matrices
load nndat5.mat

indatlength=length(xtrueS_S);
tsim=length(xtrueS_S{1,1})-1; %stores endpoint which isn't necessary

datset=zeros(8,1,1,indatlength*tsim);
solsset=zeros(1,1,4,indatlength*tsim);

n=0;
for ij=1:indatlength
    for ik=1:tsim
        n=n+1;
        up=upS_S{ij,1}{ik,1};
        ue=ueS_S{ij,1}{ik,1};
        x0=xtrueS_S{ij,1}{ik};
        datset(:,1,1,n)=x0+5*ones(8,1,1);
        solsset(1,1,:,n)=[up;ue]'+5*ones(1,4);
    end
end

dd=randperm(length(datset));

solsset=solsset(:,:,:,dd);
datset=datset(:,:,:,dd);

trainmax=1400;

data_input=datset(:,:,:,1:trainmax);
data_labels=solsset(:,:,:,1:trainmax);
data_validation_input=datset(:,:,:,trainmax+1:end);
data_validation_labels=solsset(:,:,:,trainmax+1:end);

data_generated = true;

% %test learning algorithm
%d2 = minJunderJamming(data_input,data_labels);
%d1 = minJunderJamming(rand(dmax,nx),data_labels);
%generateCostMatrices3(d1,d2);

network = [
    matrixInputLayer([8 1 1],'Normalization','none')
    
    fullyConnectedLayer(18)
    batchNormalizationLayer
    tanhLayer
    
    fullyConnectedLayer(15)
    batchNormalizationLayer
    tanhLayer
    
    fullyConnectedLayer(12)
    batchNormalizationLayer
    tanhLayer

    fullyConnectedLayer(10)
    batchNormalizationLayer
    tanhLayer
    
    fullyConnectedLayer(7)
    batchNormalizationLayer
    tanhLayer
    
    %averagePooling2dLayer(2,'Stride',2)
  
%     fullyConnectedLayer(3)
%     batchNormalizationLayer
%     tanhLayer
    
    %dropoutLayer(0.2)
    fullyConnectedLayer(4)
    regressionLayer];

miniBatchSize  = 100;
validationFrequency = floor(trainmax/miniBatchSize);
options = trainingOptions('sgdm', ...
    'MiniBatchSize',miniBatchSize, ...
    'MaxEpochs',90, ...
    'InitialLearnRate',0.0005, ...
    'LearnRateSchedule','piecewise', ...
    'LearnRateDropFactor',0.05, ...
    'LearnRateDropPeriod',30, ...
    'Shuffle','every-epoch', ...
    'ValidationData',{data_validation_input,data_validation_labels}, ...
    'ValidationFrequency',validationFrequency, ...
    'Plots','training-progress', ...
    'Verbose',false);

net = trainNetwork(data_input,data_labels,network,options);

%dd=zeros(nx,1,1,1);
%dd(:,1,1,1)=10*ones(5,1)
%predict(net,dd)