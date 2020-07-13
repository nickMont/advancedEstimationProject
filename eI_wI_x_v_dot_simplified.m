function ewxv_dot = eI_wI_x_v_dot_simplified(t,ewxv)
%NOTE: Hardcoded values for test quad
j=1;

G_body=zeros(6,1);
G_prop=zeros(6,1);
Rm_prop=zeros(6,1);
H_prop=zeros(6,1);
A=zeros(6,1);
vwind=zeros(3,1);

[e,wI,x,v]=split_ICs(ewxv,1);
qdot=[v;wI];
RBI=calculate_R_from_euler(e);
RIB=RBI';

v_rel_B=RBI*(v-vwind);

edot=wI;
xdot=v;
phi=e(1);
theta=e(2);
psi=e(3);

% %Avoid gimbal lock in dynamics at N*pi/2 (N odd)
% if abs(theta/(pi/2)-1) <= .1 || abs(theta/(pi/2)+1) <= .1
%     theta=theta+.1;
%     e
% end

m=1.5;
I_rotor=.02;
r_dis=1;
r1loc = r_dis*unit_vector([1;1;0]);
r2loc = r_dis*unit_vector([1;-1;0]);
r3loc = r_dis*unit_vector([-1;-1;0]);
r4loc = r_dis*unit_vector([-1;1;0]);
rotor_loc=[r1loc r2loc r3loc r4loc];
J=[.75 0 0
   0 .80 0
   0 0 1];
cds=[1;1;.2];
wdir=[1 -1 1 -1];
cls=[.01;.01];


cH=.001*[1;1;1;1];
cR=.005*[1;1;1;1];
cT=.1*[1;1;1;1];
kt_set=1.5;
kT=kt_set*[1;1;1;1];
g=9.81;

W=[1 0 -sin(phi)
   0 cos(phi) sin(phi)*cos(theta)
   0 -sin(phi) cos(phi)*cos(theta)];
Wt=W';
wB=W*wI;

WtJW=Wt*J*W;
C_mat=C_mat_eval(e,wI,x,v,m,J);
G_mat=[0;0;g;0;0;0];
%Output M if not invertible
if cond(WtJW)>=10^15
    fprintf('M not precisely invertible\n')
    global errorFlag
    errorFlag=true;
    WtJW=WtJW+.1*eye(3);
end
M_mat=[m*eye(3) zeros(3,3)
    zeros(3,3) WtJW];

G_body=[zeros(3,1)
    Wt*[wB(2)*wB(3)*(J(2,2)-J(3,3))
    wB(1)*wB(3)*(J(3,3)-J(1,1))
    wB(1)*wB(2)*(J(1,1)-J(2,2))]];

omegaR=[0;0;0;0];

omegaR=get_control_omegaR();

G_prop_mat=[zeros(3,4)
    Wt*I_rotor*[wB(2)*[wdir(1) wdir(2) wdir(3) wdir(4)]
    wB(1)*[wdir(1) wdir(2) wdir(3) wdir(4)]
    0 0 0 0]];
G_prop=G_prop_mat*omegaR;

A = [RIB*[-sign(v_rel_B(1))*cds(1)*v_rel_B(1)^2
    -sign(v_rel_B(2))*cds(2)*v_rel_B(2)^2
    -sign(v_rel_B(3))*cds(3)*v_rel_B(3)^2+cls(1)*v_rel_B(1)^2+cls(2)*v_rel_B(2)^2]
    Wt*[-sign(wB(1))*wB(1)^2
    -sign(wB(2))*wB(2)^2
    -sign(wB(3))*wB(3)^2]];

% %[cHx1 cHx2 ...
% % cHy1 cHy2 ...
% % cHz1 cHz2 ... ]


cHv=zeros(3,4);
cRv=zeros(3,4);
for i=1:4
    cHv(:,i)=cH(i)*(RBI*v+cross(wB,rotor_loc(:,i)));
    cRv(:,i)=cR(i)*(RBI*v+cross(wB,rotor_loc(:,i)));
end
H_prop_mat = [RIB*-cHv
    Wt*[rotor_loc(3,1)*cHv(2,1)-rotor_loc(1,1)*cHv(3,1) rotor_loc(3,2)*cHv(2,2)-rotor_loc(1,2)*cHv(3,2) rotor_loc(3,3)*cHv(2,3)-rotor_loc(1,3)*cHv(3,3) rotor_loc(3,4)*cHv(2,4)-rotor_loc(1,4)*cHv(3,4)
    rotor_loc(2,1)*cHv(3,1)-rotor_loc(3,1)*cHv(1,1) rotor_loc(2,2)*cHv(3,2)-rotor_loc(3,2)*cHv(1,2) rotor_loc(2,3)*cHv(3,3)-rotor_loc(3,3)*cHv(1,3) rotor_loc(2,4)*cHv(3,4)-rotor_loc(3,4)*cHv(1,4)
    rotor_loc(1,1)*cHv(2,1)-rotor_loc(2,1)*cHv(1,1) rotor_loc(1,2)*cHv(2,2)-rotor_loc(2,2)*cHv(1,2) rotor_loc(1,3)*cHv(2,3)-rotor_loc(2,3)*cHv(1,3) rotor_loc(1,4)*cHv(2,4)-rotor_loc(2,4)*cHv(1,4)]];
H_prop = H_prop_mat*omegaR;


Rm_mat=[zeros(3,4)
    Wt*[cRv(1,1)*wdir(1) cRv(1,2)*wdir(2) cRv(1,3)*wdir(3) cRv(1,4)*wdir(4)
    cRv(2,1)*wdir(1) cRv(2,2)*wdir(2) cRv(2,3)*wdir(3) cRv(2,4)*wdir(4)
    cRv(3,1)*wdir(1) cRv(3,2)*wdir(2) cRv(3,3)*wdir(3) cRv(3,4)*wdir(4)]];
Rm_prop=Rm_mat*omegaR;


T=zeros(6,1);

% w_vec=get_control_w();
% T_tau_to_Inertial=[RIB*[0 0 0 0; 0 0 0 0; 1 1 1 1]
%     W'*[0 1 0 0; 0 0 1 0; 0 0 0 1]];
% T=T_tau_to_Inertial*F_to_MT*forcemat(w);

T_mat= [RIB*[zeros(2,4)
    cT(1) cT(2) cT(3) cT(4)]
    Wt*[cT(1)*rotor_loc(1,1) cT(2)*rotor_loc(1,2) cT(3)*rotor_loc(1,3) cT(4)*rotor_loc(1,4)
    cT(1)*rotor_loc(2,1) cT(2)*rotor_loc(2,2) cT(3)*rotor_loc(2,3) cT(4)*rotor_loc(2,4)
    kT(1)*cT(1)*wdir(1) kT(2)*cT(2)*wdir(2) kT(3)*cT(3)*wdir(3) kT(4)*cT(4)*wdir(4)]];

%Note: omegaR here has [0;0;0;0]; as prop gyro/etc.

T=T_mat*omegaR.^2;

%T=T_mat*omegaR.^2;

% %Override T to test
% T(3)=g;
% T(4)=0;
% T(5)=0;
% T(6)=0;


qddot=inv(M_mat)*(-C_mat*qdot - G_mat + G_body + G_prop + A + Rm_prop + H_prop + T);

wdot=qddot(4:6);
vdot=qddot(1:3);

ewxv_dot=[edot;wdot;xdot;vdot];

end

