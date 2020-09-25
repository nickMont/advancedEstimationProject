function xkp1 = f_dynPur2p(x0,u,dt,vk)
ntop=length(x0)/2;
n=ntop/2;
xkp1=zeros(2*ntop,1);
xkp1(1:ntop) = [eye(n) dt*eye(n); zeros(n,n) 0.3*eye(n)]*x0(1:ntop)+...
    [eye(n)*dt^2/2;eye(n)*dt]*u(1:n)+...
    [eye(n)*dt^2/2;eye(n)*dt]*vk(1:n);
% [eye(n)*dt^2/2;eye(n)*dt]
% u(n+1:end)
xkp1(ntop+1:end) = [eye(n) dt*eye(n); zeros(n,n) 0.3*eye(n)]*x0(ntop+1:end)+...
    [eye(n)*dt^2/2;eye(n)*dt]*u(n+1:end)+...
    [eye(n)*dt^2/2;eye(n)*dt]*vk(n+1:end);

end

