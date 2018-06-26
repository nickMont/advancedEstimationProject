function xkp1 = f_dynPurQuad(x0,u,dt,vk)
set_control_omegaR(u);
xkp1 = ode45('eI_wI_x_v_dot_simplified',[0 dt],x0);
end

