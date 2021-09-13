function [Vprune,typePrune] = strictDominance(V,type,V2)
%type is 'row' or 'col'

if nargin==2
    V2=V; %permits comparison of two different matrices
end

Vprune=V;
[a0,b0]=size(V);
if strcmp(type,'row')
    typePrune=1:a0;
    contv=true;
    while contv
        contv=false;
        [a,~]=size(Vprune);
        ijHasBeenPruned=false;
        for ij=1:a
            if ~ijHasBeenPruned %only permits one removal per loop
                Vrow=Vprune(ij,:);
                V2tmp=V2(typePrune,:);
                %[a2,~]=size(Vprune);
                for ik=1:a
                    if ij~=ik && ~ijHasBeenPruned
                        Vtmp=V2tmp(ik,:);
                        diff=Vtmp-Vrow;
                        if min(diff)>=0
                            contv=true;
                            ijHasBeenPruned=true;
                            prunedIndex=typePrune(ij);
                            if ij==1
                                Vprune=Vprune(2:end,:);
                            elseif ij==a
                                Vprune=Vprune(1:end-1,:);
                            else
                                VpruneTop=Vprune(1:ij-1,:);
                                VpruneBot=Vprune(ij+1:end,:);
                                Vprune=[VpruneTop;VpruneBot];
                            end
                            typePrune=typePrune(typePrune(:)~=prunedIndex);
                        end
                    end %end ifIK~=IJ
                end %end forIK
            end %end ifPrune
        end %end forIJ
    end
elseif strcmp(type,'col')
    [Vprune,typePrune]=strictDominance(V','row',V2');
    Vprune=Vprune';
else
    error('Wrong type in strict dominance')
end



end

