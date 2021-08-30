clear;clc;

rngseedno=42; 
rng(rngseedno);

%Trains for cost-varying Q,R matrices

%Input file information--base string and number of input .mat files
file_input_string='nnTrainSets/nnQuadDyn/nnQuadDynTrain';
nummats=15;
full_nn_datfile='nnTrainSets/nnQuadDyn/nnQuadDynTrainset.mat';

last_nontraining_iteration_frac=0.98; %fraction of data to be used for training

%Mode: Read datX.mat files or load a full data file
%  0=load .mats
%  1=load full datset files
%  2=assumed already loaded
mode=1;

if mode==0
    file_to_load=[file_input_string,num2str(1),'.mat'];
    load(file_to_load);
    indatlength=iteriter;
    maximumTheoreticalLength=indatlength*nummats;
%     datset=zeros(22,1,1,indatlength*tsim*nummats);
%     solsset=zeros(1,1,4,indatlength*tsim*nummats);
    datset=zeros(16,1,1,2);
    solsset=zeros(1,1,12,2);
    
    nTrain=0; %DO NOT USE n, IT IS OVERLOADED
    for il=1:nummats
        %loadedFile=file_to_load
        for ij=1:indatlength
            %currentIteration=ij
            nTrain=nTrain+1;
            x0t=xk_S{ij};
            u0t=uS_S{ij};
            x1t=xkp1_S{ij};
            %datset(1:8,1,1,n)=x0+offset*ones(8,1,1);
            datset(1:12,1,1,nTrain)=x0t;
            datset(13:16,1,1,nTrain)=u0t;
            solsset(1,1,:,nTrain)=x1t;
            
        end
        %load next if not complete
        if il<nummats
            file_to_load=[file_input_string,num2str(il+1),'.mat'];
            load(file_to_load);
        end
    end
    save(full_nn_datfile,'solsset','datset','indatlength');
elseif mode==1
    load(full_nn_datfile);
elseif mode==2
    %do nothing
else
    fprintf('Invalid mode selection');
end
%
totalData=length(datset);
dd=randperm(totalData);

solsset=solsset(:,:,:,dd);
datset=datset(:,:,:,dd);

last_nontraining_iteration=ceil(last_nontraining_iteration_frac*totalData);
data_input=datset(:,:,:,1:last_nontraining_iteration);
data_labels=solsset(:,:,:,1:last_nontraining_iteration);
data_validation_input=datset(:,:,:,last_nontraining_iteration+1:end);
data_validation_labels=solsset(:,:,:,last_nontraining_iteration+1:end);

data_generated = true;

network = [
    matrixInputLayer([16 1 1],'Normalization','none')
    
%     fullyConnectedLayer(18)
%     batchNormalizationLayer
%     tanhLayer     
%     
%     fullyConnectedLayer(20)
%     batchNormalizationLayer
%     tanhLayer
    
    fullyConnectedLayer(22)
    batchNormalizationLayer
    tanhLayer 
    
    fullyConnectedLayer(25)
    batchNormalizationLayer
    tanhLayer 
    
    fullyConnectedLayer(22)
    batchNormalizationLayer
    tanhLayer
     
    fullyConnectedLayer(19)
    batchNormalizationLayer
    tanhLayer
    
    fullyConnectedLayer(14)
    batchNormalizationLayer
    tanhLayer

    fullyConnectedLayer(12)
    batchNormalizationLayer
    tanhLayer
    
%     fullyConnectedLayer(14)
%     batchNormalizationLayer
%     tanhLayer
    
    fullyConnectedLayer(10)
    batchNormalizationLayer
    tanhLayer
    
    %averagePooling2dLayer(2,'Stride',2)
  
%     fullyConnectedLayer(3)
%     batchNormalizationLayer
%     tanhLayer
    
    %dropoutLayer(0.2)
    fullyConnectedLayer(12)
    regressionLayer];

miniBatchSize  = 100;
validationFrequency = floor(last_nontraining_iteration/miniBatchSize);
options = trainingOptions('sgdm', ...
    'MiniBatchSize',miniBatchSize, ...
    'MaxEpochs',15, ...
    'InitialLearnRate',0.008, ...
    'LearnRateSchedule','piecewise', ...
    'LearnRateDropFactor',0.05, ...
    'LearnRateDropPeriod',50, ...
    'Shuffle','every-epoch', ...
    'ValidationData',{data_validation_input,data_validation_labels}, ...
    'ValidationFrequency',validationFrequency, ...
    'Plots','training-progress', ...
    'Verbose',false);  %'Plots','training-progress', ...
net = trainNetwork(data_input,data_labels,network,options);

testind=randsample(1:length(datset),10);
for ij=1:length(testind)
    ik=testind(ij);
    tmpd=datset(:,:,:,ik);
    tmps=solsset(:,:,:,ik);
    predictedU=predict(net,tmpd);
    Upred=double(predictedU)'
    Uopt=squeeze(tmps)
end

%dd=zeros(nx,1,1,1);
%dd(:,1,1,1)=10*ones(5,1)
%predict(net,dd)




