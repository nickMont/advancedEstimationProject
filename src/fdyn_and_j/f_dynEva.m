function xkp1 = f_dynEva(x0,u,dt,vk,params)
cd = 0;
if nargin>=5
    cd = params.cd;
end
n=length(x0)/2;
xkp1 = [eye(n) dt*eye(n); zeros(n,n) -cd*dt*eye(n)]*x0+...
    [eye(n)*dt^2/2;eye(n)*dt]*u+...
    [eye(n)*dt^2/2;eye(n)*dt]*vk;

end

