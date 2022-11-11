function R = rand0(dd,x,y)
% produces RAND of range 2*dd centered at 0
% rand0(3,9,10) produces a rand on [-3,3] of size (9,10)

if nargin==3
R=2*dd*rand(x,y)-dd;
elseif nargin==2
R=2*rand(dd,x)-1;
end

end

