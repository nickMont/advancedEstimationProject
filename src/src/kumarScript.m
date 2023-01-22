evaderUsesKumar=true;

if pursuerUsesKumar || evaderUsesKumar
    Gcontinuous=[0 0
        0 0
        1 0
        0 1];
    [Fcontinuous,Aapprx]=calculateFforKumar(Abk2,tstep,[1 0 1 0; 0 1 0 1; .01 0 -.21 0; 0 .01 0 -.21]); %NOTE:these are subtracted
end

Ppur=Peva;
%This can be shunted off to a separate function
n=0; %n is now the iteration index (yes, misuse of var names)
xTrue=[[0 0 0.3 0.1]'; [4 2 0 0]'];
xPur=xTrue;
xEva=xTrue;
axisveck=[-20 20 -5 35];

gameStateValsEva.nX=4;
gameStateValsEva.R21=zeros(2,2);
gameStateValsEva.R12=zeros(2,2);
if pursuerUsesKumar || evaderUsesKumar
    gameStateValsEva.F=Fcontinuous;
    gameStateValsEva.G1=Gcontinuous;
    gameStateValsEva.G2=Gcontinuous;
end
gameStateValsEva.W=Qnoiseeva;
gameStateValsEva.V1=Peva(1:4,1:4);
gameStateValsEva.V2=Ppur(1:4,1:4);
gameStateValsEva.R22=Reva;
gameStateValsEva.R11=Rpur;
gameStateValsEva.H1=eye(4);
gameStateValsEva.H2=eye(4);
gameStateValsEva.umax=umax;
Pvec0Eva=repmat(reshape(Qnoiseeva+Qnoisepur,[16,1]),[3 2]);
Qvec0Eva=[[reshape(Qeva, [16 1]); reshape(Qpur, [16,1]); zeros(32,1)] [reshape(Qeva, [16 1]); reshape(Qpur, [16,1]); zeros(32,1)]];

gameStateValsPur.nX=4;
gameStateValsPur.R21=zeros(2,2);
gameStateValsPur.R12=zeros(2,2);
if pursuerUsesKumar || evaderUsesKumar
    gameStateValsPur.F=Fcontinuous;
    gameStateValsPur.G1=Gcontinuous;
    gameStateValsPur.G2=Gcontinuous;
end
gameStateValsPur.W=Qnoiseeva;
gameStateValsPur.V1=Ppur(1:4,1:4);
gameStateValsPur.V2=Peva(1:4,1:4);
gameStateValsPur.R22=Rpur;
gameStateValsPur.R11=Reva;
gameStateValsPur.H1=eye(4);
gameStateValsPur.H2=eye(4);
gameStateValsPur.umax=umax;
Pvec0Pur=repmat(reshape(Qnoiseeva+Qnoisepur,[16,1]),[3 2]);
Qvec0Pur= -[[reshape(Qpur, [16 1]); reshape(Qeva, [16,1]); zeros(32,1)] [reshape(Qpur, [16 1]); reshape(Qeva, [16,1]); zeros(32,1)]];

if evaderUsesKumar || pursuerUsesKumar
tvec0pPur=[0 tstep]; tvec0pEva=[0 tstep];
tvec0qPur=[tstep 0]; tvec0qEva=[tstep 0];
for ik=1:3 %fwd/bwd pass three times
    propagateP_Pur=@(t,x) odeKumarP(t,x,Qvec0Pur,tvec0qPur,gameStateValsPur);
    [tvec0pPur,Pvec0Pur]=ode45(propagateP_Pur,[0 tstep],Pvec0Pur(:,1));
    Pvec0Pur=Pvec0Pur';
    
    propagateQ_Pur=@(t,x) odeKumarQ(t,x,Pvec0Pur,tvec0pPur,gameStateValsPur);
    [tvec0qPur,Qvec0Pur]=ode45(propagateQ_Pur,[tstep 0],Qvec0Pur(:,1));
    Qvec0Pur=Qvec0Pur';
    %NOTE: CHECK THAT OUTPUTS ARE IN THE RIGHT TIME SERIES ORDER
end

%Created separate loops for P and E for faster debugging
for ik=1:3 %fwd/bwd pass three times
    propagateP_Eva=@(t,x) odeKumarP(t,x,Qvec0Eva,tvec0qEva,gameStateValsEva);
    [tvec0pEva,Pvec0Eva]=ode45(propagateP_Eva,[0 tstep],Pvec0Eva(:,1));
    Pvec0Eva=Pvec0Eva';
    
    propagateQ_Eva=@(t,x) odeKumarQ(t,x,Pvec0Eva,tvec0pEva,gameStateValsEva);
    [tvec0qEva,Qvec0Eva]=ode45(propagateQ_Eva,[tstep 0],Qvec0Eva(:,1));
    Qvec0Eva=Qvec0Eva';
    %NOTE: CHECK THAT OUTPUTS ARE IN THE RIGHT TIME SERIES ORDER
end
end


