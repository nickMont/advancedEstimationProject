clear;clc;

Asol=[];Bsol=[];
cMMeNE=0;
cMMneNE=0;
nb=1;
xb=3;
exec=0;
p1r1=1:3;
p1r2=-2:0;
p1r3=-4:-3;
p1r4=1:3;
p2r1=p1r1;
p2r2=p1r2;
p2r3=p1r3;
p2r4=p1r4;
for p1_1=p1r1
    for p1_2=p1r2
        for p1_3=p1r3
            for p1_4=p1r4
                for p2_1=p2r1
                    for p2_2=p2r2
                        for p2_3=p2r3
                            for p2_4=p2r4
                                exec=exec+1;
                                A=[p1_1 p1_2;p1_3 p1_4];
                                B=[p2_1 p2_2;p2_3 p2_4];
                                ne2=LH2(-A,-B);
                                nR=find(ne2{1}==1);
                                nC=find(ne2{2}==1);
                                if ~isempty(nR)&&~isempty(nC)
                                    ne=[nR;nC];
                                else
                                    ne=[9001;9001];
                                end
                                mm=minimax2(A,B);
                                if mm==ne
                                    cMMeNE=cMMeNE+1;
                                end
                                if mm(1)~=ne(1) && max(ne)<9001
                                    cMMneNE=cMMneNE+1;
                                    Asol=A;
                                    Bsol=B;                                    
                                end
                            end
                        end
                    end
                end
            end
        end
    end
end
cMMeNE/exec
cMMneNE/exec
