function jj = j2Int(x,u1,u2,params)
jj = u1'*params.R21*u1 - u2'*params.R22*u2;
end

