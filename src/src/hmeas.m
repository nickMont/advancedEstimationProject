function [zk] = hmeas(xk,vk)
nx=2;
zk1 = xk(1:nx)+vk(1:nx);
zk2 = xk(nx*2+1:nx*3)+vk(1:nx);
zk = [zk1;zk2];
end

