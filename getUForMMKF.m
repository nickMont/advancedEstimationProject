function [uEst,Ru,uEvaTypeStack] = getUForMMKF(Spur,Seva,gameState,heurTypeStrucIJ)

uEvaTemp=[];
Ru=[];

if strcmp(heurTypeStrucIJ,'vmgt')
    vmgtScript;
    uEvaTemp=uEvaVMGT;
    Ru=0.05*du*eye(4);
    uEvaTypeStack='nash-vmgt';
elseif strcmp(heurTypeStrucIJ,'gt-full')
    gtfullScript;
    uEvaTemp=uEvaGT;
    Ru=0.01*du*eye(4);
    uEvaTypeStack='nash-full';
elseif strcmp(heurTypeStrucIJ,'gt-pm')
    loadPointMassControlParams;
    Ru=0.01*du*eye(4);
    uEvaTypeStack='nash-PM';
elseif strcmp(heurTypeStrucIJ,'vm')
    velmatchScript;
    uEvaTemp=uEvaVM;
    Ru=0.05*eye(4);
    uEvaTypeStack='vm';
elseif strcmp(heurTypeStrucIJ,'vmgt-heur')
    heurtype='both';
    vmgt_RA_HeurScript;
    uEvaTemp=uEvaVMGTH;
    Ru=0.10*eye(4);
    uEvaTypeStack='nash-vmgt-heur1';
elseif strcmp(heurTypeStrucIJ,'vmgt-heur2')
    heurtype='heur_only';
    vmgt_RA_HeurScript;
    uEvaTemp=uEvaVMGTH;
    Ru=0.10*eye(4);
    uEvaTypeStack{(iT-1)*nmod+ij,1}='nash-vmgt-heur2';
elseif strcmp(heurTypeStrucIJ,'other')
    uEvaTemp=omega_hover*ones(4,1);
    Ru=1*eye(4);
    uEvaTypeStack='hover';
end

uEst = uEvaTemp;

end

