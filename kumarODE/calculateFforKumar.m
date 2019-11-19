function [F,Adisc2] = calculateFforKumar(Adiscrete,dt,F_initialGuess)
%REQUIRES ITERATIVE TESTING FOR A_0
%for example, for Adiscrete=[1 1;0 .3], A_initial=[1 1; -.01 -.21] worked
%best
error=@(x) norm(expm(x*dt)-Adiscrete);
F=fminsearch(error,F_initialGuess);
Adisc2=expm(F*dt);
end

