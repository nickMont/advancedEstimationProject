clear;clc;

rngseedno=40;  %good at 10
rng(rngseedno);

%Trains for cost-varying Q,R matrices

%Input file information--base string and number of input .mat files
file_input_string='nnTrainSets/nnVarDyn';
nummats=25;
full_nn_datfile='nn1v1HighControl.mat';

last_nontraining_iteration=11500;  %good at 1450

%Mode: Read datX.mat files or load a full data file
%  0=load .mats
%  1=load full datset files
%  2=assumed already loaded
mode=1;

if mode==0
    file_to_load=[file_input_string,num2str(1),'.mat'];
    load(file_to_load);
    indatlength=length(xtrueS_S);
    tsim=length(xtrueS_S{1,1})-1; %stores endpoint which isn't necessary
    datset=zeros(22,1,1,indatlength*tsim*nummats);
    solsset=zeros(1,1,4,indatlength*tsim*nummats);
    
    n=0;
    offset=0;
    offset_u=0;
    offx=0;
    offv=0;
    offset_multiply_qqrr=1;
    offset_vec=[offx;offx;offv;offv; offx;offx;offv;offv];
    for il=1:nummats
        %loadedFile=file_to_load
        for ij=1:indatlength
            %currentIteration=ij
            qqrr=qrS_S{ij,1};
            for ik=1:tsim
                n=n+1;
                up=upS_S{ij,1}{ik,1};
                ue=ueS_S{ij,1}{ik,1};
                x0=xtrueS_S{ij,1}{ik};
                %datset(1:8,1,1,n)=x0+offset*ones(8,1,1);
                datset(1:8,1,1,n)=x0+offset_vec;
                datset(9:22,1,1,n)=qqrr*offset_multiply_qqrr;
                solsset(1,1,:,n)=[up;ue]'+offset_u*ones(1,4);
            end
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

dd=randperm(length(datset));

solsset=solsset(:,:,:,dd);
datset=datset(:,:,:,dd);

data_input=datset(:,:,:,1:last_nontraining_iteration);
data_labels=solsset(:,:,:,1:last_nontraining_iteration);
data_validation_input=datset(:,:,:,last_nontraining_iteration+1:end);
data_validation_labels=solsset(:,:,:,last_nontraining_iteration+1:end);

data_generated = true;

% %test learning algorithm
%d2 = minJunderJamming(data_input,data_labels);
%d1 = minJunderJamming(rand(dmax,nx),data_labels);
%generateCostMatrices3(d1,d2);

network = [
    matrixInputLayer([22 1 1],'Normalization','none')
    
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

    fullyConnectedLayer(8)
    batchNormalizationLayer
    tanhLayer
    
    fullyConnectedLayer(6)
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
validationFrequency = floor(last_nontraining_iteration/miniBatchSize);
options = trainingOptions('sgdm', ...
    'MiniBatchSize',miniBatchSize, ...
    'MaxEpochs',120, ...
    'InitialLearnRate',0.015, ...
    'LearnRateSchedule','piecewise', ...
    'LearnRateDropFactor',0.01, ...
    'LearnRateDropPeriod',100, ...
    'Shuffle','every-epoch', ...
    'ValidationData',{data_validation_input,data_validation_labels}, ...
    'ValidationFrequency',validationFrequency, ...
    'Plots','training-progress', ...
    'Verbose',false);

net = trainNetwork(data_input,data_labels,network,options);

%dd=zeros(nx,1,1,1);
%dd(:,1,1,1)=10*ones(5,1)
%predict(net,dd)