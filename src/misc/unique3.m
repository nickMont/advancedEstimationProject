function v_unique = unique3(vecin,epsilon,type)
if nargin<=1
    epsilon=1e-6;
end
if nargin<=2
    type='col';
end

v_unique=[];

if strcmp(type,'col')
    v_unique=vecin(:,1);
    [~,a]=size(vecin);
    for ij=2:a
        this_v=vecin(:,ij);
        [~,b]=size(v_unique);
        flag_is_unique=true;
        for ik=1:b
            if norm(v_unique(:,ik)-this_v)<epsilon
                flag_is_unique=false;
            end
        end
        if flag_is_unique
            v_unique=[v_unique this_v];
        end
    end
elseif strcmp(type,'row')
    v_unique=vecin(1,:);
    [a,~]=size(vecin);
    for ij=2:a
        this_v=vecin(ij,:);
        [b,~]=size(v_unique);
        flag_is_unique=true;
        for ik=1:b
            if norm(v_unique(ik,:)-this_v)<epsilon
                flag_is_unique=false;
            end
        end
        if flag_is_unique
            v_unique=[v_unique; this_v];
        end
    end
end
end

