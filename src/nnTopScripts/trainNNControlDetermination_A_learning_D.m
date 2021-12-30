% clear;clc;

rngseedno=58; 
rng(rngseedno);

%Trains for cost-varying Q,R matrices

flagSaveLabels=true;
flagQuitWithoutTraining=false;
flagValidateSetWithClassifier=false;

targetIndexOffset = 100; %offset target index from control type index
% This makes the switch from numerical indices to categoricals easier for
% Matlab.

%NOTE USE nnTrainSets/nnDetermineControlType/pointMassTargetPurVM_loaded
%ONLY IF NECESSARY DUE TO OLD WIPEOFF BUG

% %Input file information--base string and number of input .mat files
file_input_string='nnTrainSets/nnDetermineControlType_X_Target_aLd/mat';
nummats=15;
full_nn_datfile='nnTrainSets/nnDetermineControlType_X_Target_aLd/full.mat';
full_label_datfile='nnTrainSets/nnDetermineControlType_X_Target_aLd/fullWithLabels.mat';
last_nontraining_iteration_frac=0.95; %fraction of data to be used for training

% file_input_string='nnTrainSets/temp2';
% nummats=1;
% full_nn_datfile='nnTrainSets/tempout2.mat';
% full_label_datfile='nnTrainSets/tempout2.mat';
% last_nontraining_iteration_frac=0; %fraction of data to be used for training


%Mode: Read datX.mat files or load a full data file
%  0=load and combine .mats
%  1=load full datset files
%  2=assumed already loaded
mode=0;

if mode==0
    file_to_load=[file_input_string,num2str(1),'.mat'];
%     file_to_load=file_input_string;
    load(file_to_load);
    indatlength=numNNiter;
    %maximumTheoreticalLength=indatlength*nummats;
    datset=zeros(1295,1,1,1);
    solsset=cell(1,1);
    uP=zeros(82,1);
    uE=zeros(82,1);
    
    nTrain=0; %DO NOT USE n, IT IS OVERLOADED
    for il=1:nummats
        %loadedFile=file_to_load
        for ij=1:numNNiter
            %currentIteration=ij
            nTrain=nTrain+1;
            x0t=xStore_S{ij};
%             x0t=x0t(1:8,:);
            [at,bt]=size(x0t);
            x1t=controlTypeStore{ij};
            qrTemp = qrStore{1,ij};
            qrCosts = qrTemp(1:end-2);
            xtar1 = targetPossibleStore{ij}{1};
            xtar2 = targetPossibleStore{ij}{2};
            datset(:,1,1,nTrain)=[qrCosts;xtar1;xtar2;reshape(x0t,[at*bt,1])];
            controlType2String = num2str(x1t);
            targetIndex2String = num2str(targetTrueIndexStore{ij}+targetIndexOffset);
            solsset{nTrain,1}=[controlType2String targetIndex2String];
%             uBp=uStore{1,ij};
%             uBe=uStore{2,ij};
%             uP(:,nTrain)=reshape(uBp,[82 1]);
%             uE(:,nTrain)=reshape(uBe,[82 1]);
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
% if mode==0
%     uP=uP(:,dd);
%     uE=uE(:,dd);
% end

last_nontraining_iteration=ceil(last_nontraining_iteration_frac*totalData);
data_input=datset(:,:,:,1:last_nontraining_iteration);
data_labels=solsset(1:last_nontraining_iteration,:);
% if mode==0
% data_uP=uP(:,1:last_nontraining_iteration); data_uE=uE(:,1:last_nontraining_iteration);
% end
data_validation_input=datset(:,:,:,last_nontraining_iteration+1:end);
data_validation_labels=solsset(last_nontraining_iteration+1:end,:);
% if mode==0
% data_validation_uP=uP(:,last_nontraining_iteration+1:end); data_validation_uE=uE(:,last_nontraining_iteration+1:end);
% end

if flagSaveLabels && mode==0
    save(full_label_datfile);
end

data_generated = true;

if ~flagQuitWithoutTraining
network = [
    matrixInputLayer([1295 1 1],'Normalization','none')
    
%     fullyConnectedLayer(18)
%     batchNormalizationLayer
%     tanhLayer     
%     
%     fullyConnectedLayer(20)
%     batchNormalizationLayer
%     tanhLayer
    
    fullyConnectedLayer(1000)
    batchNormalizationLayer
    tanhLayer 

    fullyConnectedLayer(800)
    batchNormalizationLayer
    tanhLayer 
    
    fullyConnectedLayer(600)
    batchNormalizationLayer
    tanhLayer 
    
    fullyConnectedLayer(400)
    batchNormalizationLayer
    tanhLayer
     
    fullyConnectedLayer(200)
    batchNormalizationLayer
    tanhLayer
    
    fullyConnectedLayer(100)
    batchNormalizationLayer
    tanhLayer
    
    fullyConnectedLayer(50)
    batchNormalizationLayer
    tanhLayer

    fullyConnectedLayer(40)
    batchNormalizationLayer
    tanhLayer
    
%     fullyConnectedLayer(14)
%     batchNormalizationLayer
%     tanhLayer
    
    fullyConnectedLayer(20)
    batchNormalizationLayer
    tanhLayer
    
    %averagePooling2dLayer(2,'Stride',2)
  
    fullyConnectedLayer(15)
    batchNormalizationLayer
    tanhLayer
    
    %dropoutLayer(0.2)
    fullyConnectedLayer(6)
    softmaxLayer
    classificationLayer];

miniBatchSize  = 100;
validationFrequency = floor(last_nontraining_iteration/miniBatchSize);
options = trainingOptions('sgdm', ...
    'MiniBatchSize',miniBatchSize, ...
    'MaxEpochs',80, ...
    'InitialLearnRate',0.005, ...
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

if flagValidateSetWithClassifier
[~,~,~,maxDat]=size(data_validation_input);
Pset=[];
for iL=1:maxDat
    tmp = classify(net,data_validation_input(:,:,:,iL));
    Pset=[Pset; tmp];
end

PsetN=str2num(char(Pset));
valSetN=str2num(char(data_validation_labels));
cErr=abs(PsetN-valSetN);
errorIndices=find(cErr~=0)
errorControlTypeI2 = find(mod(cErr(errorIndices),1000)==0)
errorTargetOnlyI2 = find(cErr(errorIndices<=500))
errorBoth = errorIndices(setdiff(1:end,[errorTargetOnlyI2; errorControlTypeI2]))

end



