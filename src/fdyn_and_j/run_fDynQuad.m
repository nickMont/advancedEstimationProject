function x2 = run_fDynQuad(inputs)
% Allows f_dynPurQuad and f_dynEvaQuad to be run as
%   feval('run_fDynQuad',input')

% Useful for future standardization of code

if isfield(inputs,'vk')
    x2 = feval(inputs.fname,inputs.state,inputs.u,inputs.dt,inputs.vk);
else
    x2 = feval(inputs.fname,inputs.state,inputs.u,inputs.dt,zeros(6,1));
end

end

