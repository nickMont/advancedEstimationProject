clear;clc;

rngseedno=43; 
rng(rngseedno);

%Trains for cost-varying Q,R matrices

flagSaveLabels=true;
flagQuitWithoutTraining=false;

%NOTE USE nnTrainSets/nnDetermineControlType/pointMassTargetPurVM_loaded
%ONLY IF NECESSARY DUE TO OLD WIPEOFF BUG

% %Input file information--base string and number of input .mat files
file_input_string='nnTrainSets/nnDetermineControlType/pointMassTargetPurVM/dat';
nummats=8;
full_nn_datfile='nnTrainSets/nnDetermineControlType/pointMassTargetPurVM/full.mat';
full_label_datfile='nnTrainSets/nnDetermineControlType/pointMassTargetPurVM/fullWithLabels.mat';
last_nontraining_iteration_frac=0.98; %fraction of data to be used for training

% file_input_string='nnTrainSets/temp2';
% nummats=1;
% full_nn_datfile='nnTrainSets/tempout2.mat';
% full_label_datfile='nnTrainSets/tempout2.mat';
% last_nontraining_iteration_frac=0; %fraction of data to be used for training


%Mode: Read datX.mat files or load a full data file
%  0=load and combine .mats
%  1=load full datset files
%  2=assumed already loaded
mode=1;

if mode==0
    file_to_load=[file_input_string,num2str(1),'.mat'];
%     file_to_load=file_input_string;
    load(file_to_load);
    indatlength=numNNiter;
    %maximumTheoreticalLength=indatlength*nummats;
    datset=zeros(356,1,1,1);
    solsset=cell(1,1);
    uP=zeros(82,1);
    uE=zeros(82,1);
    
    nTrain=0; %DO NOT USE n, IT IS OVERLOADED
    for il=1:nummats
        %loadedFile=file_to_load
        for ij=1:numNNiter
            %currentIteration=ij
            nTrain=nTrain+1;
            x0t=xStore{ij};
            x0t=x0t(1:8,:);
            [at,bt]=size(x0t);
            x1t=controlTypeStore{ij};
            datset(:,1,1,nTrain)=[qrStore{1,ij};reshape(x0t,[at*bt,1])];
            solsset{nTrain,1}=num2str(x1t);
            uBp=uStore{1,ij};
            uBe=uStore{2,ij};
            uP(:,nTrain)=reshape(uBp,[82 1]);
            uE(:,nTrain)=reshape(uBe,[82 1]);
        end
        if il<nummats
            file_to_load=[file_input_string,num2str(il+1),'.mat'];
            load(file_to_load);
        end
    end
    save(full_nn_datfile,'solsset','datset','indatlength','rngseedno');
elseif mode==1
    load(full_nn_datfile);
elseif mode==2
    %do nothing
else
    fprintf('Invalid mode selection');
end
%
% datset=squeeze(datset)';

[~,~,~,totalData]=size(datset);
dd=randperm(totalData);

solsset=categorical(solsset(dd,:));
datset=datset(:,:,:,dd);
uP=uP(:,dd);
uE=uE(:,dd);

last_nontraining_iteration=ceil(last_nontraining_iteration_frac*totalData);
data_input=datset(:,:,:,1:last_nontraining_iteration);
data_labels=solsset(1:last_nontraining_iteration,:);
data_uP=uP(:,1:last_nontraining_iteration); data_uE=uE(:,1:last_nontraining_iteration);
data_validation_input=datset(:,:,:,last_nontraining_iteration+1:end);
data_validation_labels=solsset(last_nontraining_iteration+1:end,:);
data_validation_uP=uP(:,last_nontraining_iteration+1:end); data_validation_uE=uE(:,last_nontraining_iteration+1:end);

if flagSaveLabels
    save(full_label_datfile);
end

data_generated = true;

if ~flagQuitWithoutTraining
network = [
    matrixInputLayer([356 1 1],'Normalization','none')
    
%     fullyConnectedLayer(18)
%     batchNormalizationLayer
%     tanhLayer     
%     
%     fullyConnectedLayer(20)
%     batchNormalizationLayer
%     tanhLayer
    
    fullyConnectedLayer(100)
    batchNormalizationLayer
    tanhLayer 
    
    fullyConnectedLayer(70)
    batchNormalizationLayer
    tanhLayer 
    
    fullyConnectedLayer(50)
    batchNormalizationLayer
    tanhLayer
     
    fullyConnectedLayer(30)
    batchNormalizationLayer
    tanhLayer
    
    fullyConnectedLayer(20)
    batchNormalizationLayer
    tanhLayer
    
    fullyConnectedLayer(15)
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
  
    fullyConnectedLayer(7)
    batchNormalizationLayer
    tanhLayer
    
    %dropoutLayer(0.2)
    fullyConnectedLayer(3)
    softmaxLayer
    classificationLayer];

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

% testind=randsample(1:length(datset),10);
% for ij=1:length(testind)
%     ik=testind(ij);
%     tmpd=datset(:,:,:,ik);
%     tmps=solsset(:,:,:,ik);
%     predictedU=predict(net,tmpd);
%     Upred=double(predictedU)'
%     Uopt=squeeze(tmps)
% end

%dd=zeros(nx,1,1,1);
%dd(:,1,1,1)=10*ones(5,1)
%predict(net,dd)
end



