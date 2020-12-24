function xkp1 = f_dynEva(x0,u,dt,vk,empty)
n=length(x0)/2;
xkp1 = [eye(n) dt*eye(n); zeros(n,n) 0.3*eye(n)]*x0+...
    [eye(n)*dt^2/2;eye(n)*dt]*u+...
    [eye(n)*dt^2/2;eye(n)*dt]*vk;

% cd=0.8;
% A=[zeros(n,n) eye(n); zeros(n,n) -cd*eye(n)];
% F=expm(A*dt);

end

