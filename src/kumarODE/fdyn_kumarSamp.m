function xdot = fdyn_kumarSamp(t,x,params)
xdot = params.F*x+params.G1*params.u1-params.G2*params.u2;
end

