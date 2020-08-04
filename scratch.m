clear;clc;

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

dd=f_dynPurQuad(zeros(12,1),4.95*ones(4,1),1,zeros(2,1));
dd(7:9)