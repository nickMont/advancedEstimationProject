function [uOut,Jo] = optimizeGivenEnemyControl(Sinput)
uOut=[];
Jo=[];

if strcmp(Sinput.type,'fmincon')
    % Takes as input:
    %   Sinput.uOpponentActual, Sinput.Sself, Sinput.Sopp, Sinput.gameState, 
    %   Sinput.selfState, Sinput.oppState, Sinput.type, Sinput.uSelfGuess
    nU=length(Sinput.uSelfGuess);
    xOpp = feval(Sinput.Sopp.fname,Sinput.oppState,Sinput.uOpponentActual,Sinput.gameState.dt,zeros(6,1));
    %Jpur=feval(Spur.Jname,xPurCell{iP,iE},xEvaCell{iP,iE},uPcell{iP,iE},uEcell{iP,iE},Spur.Jparams);
    J=@(u) feval(Sinput.Sself.Jname,...
        [Sinput.selfState feval(Sinput.Sself.fname,Sinput.selfState,u,Sinput.gameState.dt,zeros(6,1))],...
        [Sinput.oppState xOpp],u,Sinput.uOpponentActual,Sinput.Sself.Jparams);
    [uOut,Jo]=fmincon(J,Sinput.uSelfGuess,[],[],[],[],zeros(nU,1),Sinput.Sself.uLmax*ones(nU,1));
    
elseif strcmp(Sinput.type,'discretize')
    if strcmp(Sinput.player,'pur')
        SevaTemp=Sinput.Seva;
        SpurTemp=Sinput.Spur;
        umax=Sinput.umax;
        utemp=Sinput.utemp;
        uvec=Sinput.uvec;
        if strcmp(SpurTemp.controlType,'vmquad')
            for ik=1:length(uvec)
                SpurTemp.uMat{ik}=umax*uvec(ik);
            end
        elseif strcmp(SpurTemp.controlType,'gt_overx')
            for ik=1:length(utemp)
                SpurTemp.uMat{ik}=utemp(:,ik);
            end
        else
            error('Unrecognized control type');
        end
        [~,~,~,uOut]=f_dyn2(SpurTemp,SevaTemp,Sinput.gameState,zeros(4,1));
    elseif strcmp(Sinput.player,'eva')
        SevaTemp=Sinput.Seva;
        SpurTemp=Sinput.Spur;
        umax=Sinput.umax;
        utemp=Sinput.utemp;
        uvec=Sinput.uvec;
        if strcmp(SevaTemp.controlType,'vmquad')
            for ik=1:length(uvec)
                SevaTemp.uMat{ik}=umax*uvec(ik);
            end
        elseif strcmp(SevaTemp.controlType,'gt_overx')
            for ik=1:length(utemp)
                SevaTemp.uMat{ik}=utemp(:,ik);
            end
        else
            error('Unrecognized control type');
        end
        [~,~,~,~,uOut]=f_dyn2(SpurTemp,SevaTemp,Sinput.gameState,zeros(4,1));
    else
        error('Unrecognized player type in Sinput')
    end
else
    error('wrong type in optimizeGivenEnemyControl')
end

end

