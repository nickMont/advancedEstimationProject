function J = minpair(pairings,params)
xE=params.xE;
xP=params.xP;
nE=params.nEva;
nP=params.nPur;
nx=params.nx;

a=length(pairings)/2;

J=0;

for ik=1:a
    evaIndex=pairings(2*ik);
    purIndex=pairings(2*ik-1);
    
    dx=xE{evaIndex}(1:nx)-xP{purIndex}(1:nx);
    J=J+norm(dx);
end

untrackedEvaders=setdiff(1:nE,pairings(2:2:end));
J=J+1000*length(untrackedEvaders);

end

