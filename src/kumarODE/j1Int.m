function jj = j1Int(x,u1,u2,params)
jj = u1'*params.R11*u1 - u2'*params.R21*u2;
end

