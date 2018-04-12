function uFeas = generateGameControls(possibleControls,N,flag)
%flag=2: all controls are the same, use combnk
%flag=1: some controls are different, calculate manually

if flag==2
%   NOTE: COMBVEC MOVED TO NN TOOLBOX
%     allFeas=combvec(possibleControls.quad{1}.theta,N);
%     [a,b]=size(allFeas);
%     for ij=1:a
%         for ik=1:b
%             uFeas.quad{b}=1;
%             n=n+1;
%         end
%     end
elseif flag==1
    
    
else
    uFeas=9001;
    
end

% n0=1;
% for ij=1:N
%     n0=n0*length(possibleControls.quad{ij}.theta);
% end
% allFeas=zeros(n0,N);



end