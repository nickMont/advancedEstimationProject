% clear;clc;

% A=[1,5,6,9,12];
% B= [1,2,3,4,5,6];
% C= [3,18,27,69,72];
% [ii,jj,kk]=meshgrid(A,B,C);
% ii=permute(ii,[1 3 2]);
% jj=permute(jj,[2 1 3]);
% kk=permute(kk,[3  2 1]);
% out=[ii(:) jj(:) kk(:)];
% 
% 
% A=[1 0; 1 2];
% B= [1,2,3,4,5,6];
% C= [3,18,27,69,72];
% [ii,jj,kk]=meshgrid(A,B,C);
% ii=permute(ii,[1 3 2]);
% jj=permute(jj,[2 1 3]);
% kk=permute(kk,[3  2 1]);
% out=[ii(:) jj(:) kk(:)]
% numel(A)*numel(B)*numel(C)
% size(out)

% dd=f_dynPurQuad(zeros(12,1),4.95*ones(4,1),1,zeros(2,1));
% dd(7:9)

z=xTrue+chol(QnoiseStack)'*rand(8,1);
zMeas=z(5:8);

lambdaTemp=-1*ones(nmod,1);
for ij=1:nmod
    uEvaMMF=uEvaTempStack{ij};
    RuMMF=RuStack{ij};
    
    Paug=PhatE(:,:,ij); Qk=RuMMF;
    if min(eig(Paug))>0 %check for impossible models
        % Propagate
        [xhatEp1,Pp1]=ukfPropagate(xhatE(:,ij),PhatE(:,:,ij),uEvaMMF,RuMMF,dt,'f_dynEva');
        
        % Measure
        [xhatE(:,ij),PhatE(:,:,ij),nu,Sk,Wk]=kfMeasure(xhatEp1,Pp1,zMeas,eye(length(xTrue(5:8))),Rk);
        Skt=triu(Sk); Skt=Skt'+Skt; Skt(1:5:end)=diag(Sk); %forces symmetry
        normpEval= mvnpdf(nu,zeros(length(xTrue(5:8)),1),Skt);
    else
        normpEval=1e-20;
    end
    if normpEval<=1e-8
        normpdf_eval=1e-8;
    end
    lambdaTemp(ij)=normpEval;
end
muTemp=mu;
for ij=1:nmod
    muTemp(ij)=lambdaTemp(ij)*mu(ij)/dot(lambdaTemp,mu);
    if muTemp(ij)<1e-8
        muTemp(ij)=1e-8;
    end
end
mu=muTemp/sum(muTemp)
muHist=[muHist mu];



    
    
    