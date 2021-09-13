function R = rand0(dd,x,y)
% produces RAND of range 2*dd centered at 0
% rand0(3,9,10) produces a rand on [-3,3] of size (9,10)

R=2*dd*rand(x,y)-dd;

end

