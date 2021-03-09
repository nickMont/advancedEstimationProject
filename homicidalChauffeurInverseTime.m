function stateDot = homicidalChauffeurInverseTime(T,state,params)

%Dynamics here expressed in inverse time
% see Pachter&Coates 2018: The Classical Homicidal Chauffeur Game

%"Dot" here represents differentiation in inverse time, see paper

% Tmax = 2*(pi-phi(0))

x = state(1);
y = state(2);
lambdaX = state(3);
lambdaY = state(4);
lambdaDotX = state(5);
lambdaDotY = state(6);

mu=params.mu;
phi=params.phi;

S = lambdaY*x - lambdaX*y;
u=0;
if abs(S)<1e-10
    u=0;
else
    u=sign(S);
end

lambdaM = sqrt(lambdaX^2+lambdaY^2);
Psi = atan2(-lambdaX/lambdaM,-lambdaY/lambdaM);

lambdaDotDotX = -lambdaX;
lambdaDotDotY = -lambdaY;

xDot = y-mu*sin(phi+T);
yDot = -x+1-mu*cos(phi+T);


stateDot=[xDot
    yDot
    lambdaDotX
    lambdaDotY
    lambdaDotDotX
    lambdaDotDotY];
end

