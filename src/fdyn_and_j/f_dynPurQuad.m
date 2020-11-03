function xkp1 = f_dynPurQuad(x0,u,dt,vk)
set_control_omegaR(u);
if length(vk)==4
    set_control_omegaR(u+vk);
elseif length(vk)==7
    set_control_omegaR(u+vk(4:7));
end
[t,x] = ode45('eI_wI_x_v_dot_simplified',[0 dt],x0);
xkp1=x(end,:)';
global errorFlag
if errorFlag
    fprintf('Err in pursuer at u:')
    uErr=u
    errorFlag=false;
    xkp1=9001*ones(12,1);
end

end

