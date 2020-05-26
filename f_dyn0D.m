function xkp1 = f_dyn0D(x0,u,dt,vk)
n=length(x0)/2;
xkp1 = [eye(n) dt*eye(n); zeros(n,n) eye(n)]*x0+...
    [eye(n)*dt^2/2;eye(n)*dt]*u+...
    [eye(n)*dt^2/2;eye(n)*dt]*vk;
% dx=[eye(n) dt*eye(n); zeros(n,n) eye(n)]*x0
% du=[eye(n)*dt^2/2;eye(n)*dt]*u
% dv=[eye(n)*dt^2/2;eye(n)*dt]*vk
% cd=0.8;
% A=[zeros(n,n) eye(n); zeros(n,n) -cd*eye(n)];
% F=expm(A*dt);

end

