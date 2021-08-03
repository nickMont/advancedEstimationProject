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

% z=xTrue+chol(QnoiseStack)'*rand(8,1);
% zMeas=z(5:8);
% 
% lambdaTemp=-1*ones(nmod,1);
% for ij=1:nmod
%     uEvaMMF=uEvaTempStack{ij};
%     RuMMF=RuStack{ij};
%     
%     Paug=PhatE(:,:,ij); Qk=RuMMF;
%     if min(eig(Paug))>0 %check for impossible models
%         % Propagate
%         [xhatEp1,Pp1]=ukfPropagate(xhatE(:,ij),PhatE(:,:,ij),uEvaMMF,RuMMF,dt,'f_dynEva');
%         
%         % Measure
%         [xhatE(:,ij),PhatE(:,:,ij),nu,Sk,Wk]=kfMeasure(xhatEp1,Pp1,zMeas,eye(length(xTrue(5:8))),Rk);
%         Skt=triu(Sk); Skt=Skt'+Skt; Skt(1:5:end)=diag(Sk); %forces symmetry
%         normpEval= mvnpdf(nu,zeros(length(xTrue(5:8)),1),Skt);
%     else
%         normpEval=1e-20;
%     end
%     if normpEval<=1e-8
%         normpdf_eval=1e-8;
%     end
%     lambdaTemp(ij)=normpEval;
% end
% muTemp=mu;
% for ij=1:nmod
%     muTemp(ij)=lambdaTemp(ij)*mu(ij)/dot(lambdaTemp,mu);
%     if muTemp(ij)<1e-8
%         muTemp(ij)=1e-8;
%     end
% end
% mu=muTemp/sum(muTemp)
% muHist=[muHist mu];


% % testing different expm calcs
% tstep=1;
% tmax=4;
% dt = tstep;
% 
% xTrue=randn(4,1);
% Gcontinuous1=[0 0
%               0 0
%               1 0
%               0 1];
% Gcontinuous2=[0 0
%               0 0
%               1 0
%               0 1];          
% Fcontinuous=[0 0 1 0
%              0 0 0 1
%              0 0 0 0
%              0 0 0 0];
% nx = length(Fcontinuous);
% 
% u1=randn(2,1)
% u2=randn(2,1)
% 
% %calculate state transition matrix here
% A=expm(Fcontinuous*dt);
% %Note: B,u continuous over time step, so zero-state response is just
% %  the (integral of the state transition matrix)*B*u
% B1 = (expm(Fcontinuous*dt)-eye(4))*inv(Fcontinuous)*Gcontinuous1;
% B2 = (expm(Fcontinuous*dt)-eye(4))*inv(Fcontinuous)*Gcontinuous2;
% Gstack = [Gcontinuous1 -Gcontinuous2];
% Bstack = (expm(Fcontinuous*dt)-eye(4))*(Fcontinuous\Gstack);
% % Bstack2 = A*(-A*inv(Fcontinuous
% 
% z2 = zeros(2,2);
% 
% [V,J]= jordan(Fcontinuous);
% BB = J(1:2,1:2);
% CC = J(3:4,3:4);
% int_eTBdt  = dt*eye(2);
% Bstack2 = inv(V) * [int_eTBdt z2; z2 inv(CC)*(expm(CC*dt)-eye(2))] * V;
% 
% gameStateVals.F=Fcontinuous;
% gameStateVals.G1=Gcontinuous1;
% gameStateVals.G2=Gcontinuous2;
% 
% paramsSim = gameStateVals;
% paramsSim.u1 = u1;
% paramsSim.u2 = u2;
% os = @(t,x) fdyn_kumarSamp(t,x,paramsSim);
% [t2,x2]=ode45(os,[0 0+tstep],xTrue);
% 
% Bstack3 = [dt^2/2 0 -dt^2/2 0
%     0 dt^2/2 0 -dt^2/2
%     dt 0 -dt 0
%     0 dt 0 -dt];
%     
% xEndC = x2(end,:)'
% xEndD = A*xTrue + B1*u1 - B2*u2;
% xEndDstack = A*xTrue + Bstack*[u1;u2];
% xEndDstack2 = A*xTrue + Bstack2*[u1;u2];
% xEndDstack3 = A*xTrue + Bstack3*[u1;u2]




