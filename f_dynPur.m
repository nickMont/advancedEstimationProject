function xkp1 = f_dynPur(x0,u,dt,vk)
n=length(x0)/2;
xkp1 = [eye(n) dt*eye(n); zeros(n,n) eye(n)]*x0+...
    [eye(n)*dt^2/2;eye(n)*dt]*u+...
    [eye(n)*dt^2/2;eye(n)*dt]*vk;

end

