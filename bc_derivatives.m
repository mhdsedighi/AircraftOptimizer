%altitude=5000ft
% U0=340;
U0=200;
theta0=0;
phi0=0;
alpha0=0;
beta0=0;


phi0=phi0*pi/180;
theta0=theta0*pi/180;


g=32.1740;

s=280;
c=6.5;
b=46;

q=118.3;
X_cg=0.26;

u0=U0;
v0=U0*alpha0;
w0=U0*beta0;

m=7000;
Ixx=10085;
Iyy=15148;
Izz=23046;
Ixz=1600;
Iyz=0;
Ixy=0;



CD0=0.0270;
CD_u=0;
CD_a=0.131;
CTX_u=-0.0596;
CL0=0.201;
CL_u=0.02;
CL_a=5.48;
CL_ad=2.5;
CL_q=8.1;


Cm0=0.05;
Cm_u=0;
Cm_a=-1.89;
Cm_ad=-9.1;
Cm_q=-34.0;
CmT_u=0;
CmT_a=0;



CD_dE=0;
CL_dE=0.58;
Cm_dE=-1.9;
CD_iH=0;
CL_iH=1.3;
Cm_iH=-3.9;
CmT0=0;

Cl_b=-0.13;
Cl_p=-0.5;
Cl_r=0.14;
CY_b=-0.59;
CY_p=-0.19;
CY_r=0.39;
Cn_b=0.080;
CnT_b=0;
Cn_p=-0.019;
Cn_r=-0.197;

Cl_dA=0.156;
Cl_dR=0.0109;
CY_dA=0;
CY_dR=0.148;
Cn_dA=-0.0012;
Cn_dR=-0.0772;

CTX0=0;

%%%%%%%%%%%%%%%%%%%%%%%%%%


I=[Ixx 0 -Ixz;0 Iyy 0;-Ixz 0 Izz];
II=inv(I);

%%%

CX0=-CD0;
CX_u=-CD_u;
CX_w=-CD_a/U0;
CY_v=CY_b/U0;
CZ0=-CL0;
CZ_u=-CL_u;
CZ_w=-CL_a/U0;
CZ_q=-CL_q;
Cm_w=Cm_a/U0;
Cl_v=Cl_b/U0;
Cn_v=Cn_b/U0;
CX_dE=-CD_dE;
CZ_dE=-CL_dE;
CZ_wd=-CL_ad/U0;
Cm_wd=Cm_ad/U0;
%%%

X_u=(q*s/m)*((2*u0/U0^2)*CX0+CX_u);
XT_u=(q*s/m)*((2*u0/U0^2)*CTX0+CTX_u);
X_w=(q*s/m)*((2*w0/U0^2)*CX0+CX_w);
X_dE=(q*s/m)*CX_dE;
X_q=0;
X_dT=0;



Z_u=(q*s/m)*((2*u0/U0^2)*CZ0+CZ_u);
Z_w=(q*s/m)*((2*w0/U0^2)*CZ0+CZ_w);
Z_wd=(q*s/m/2/U0)*c*CZ_w;  %%%
Z_q=(q*s/m/2/U0)*c*CZ_q;
Z_dE=(q*s/m)*CZ_dE;
Z_dT=0;


Y_v=(q*s/m)*CY_v;
Y_p=(q*s/m/2/U0)*b*CY_p;
Y_r=(q*s/m/2/U0)*b*CY_r;
Y_dA=(q*s/m)*CY_dA;
Y_dR=(q*s/m)*CY_dR;

M_u=(q*s)*c*Cm_u;
MT_u=(q*s)*c*CmT_u;
M_w=(q*s)*c*Cm_w;
MT_w=0;
M_wd=(q*s/2/U0)*c^2*Cm_wd; %%
M_q=(q*s/2/U0)*c^2*Cm_q;
M_dE=(q*s)*c*Cm_dE;
M_dT=0;
M_theta=0;

L_v=(q*s)*b*Cl_v;
L_p=(q*s/2/U0)*b^2*Cl_p;
L_r=(q*s/2/U0)*b^2*Cl_r;
L_dA=(q*s)*b*Cl_dA;
L_dR=(q*s)*b*Cl_dR;


N_v=(q*s)*b*Cn_v;
NT_v=0;
N_vd=0;
N_p=(q*s/2/U0)*b^2*Cn_p;
N_r=(q*s/2/U0)*b^2*Cn_r;
N_dA=(q*s)*b*Cn_dA;
N_dR=(q*s)*b*Cn_dR;
N_dT=0;




MAT=II*[0 L_v 0;M_u+MT_u 0 M_w+MT_w;0 N_v+NT_v 0];
L_v=MAT(1,2);
M_u=MAT(2,1);
M_w=MAT(2,3);
N_v=MAT(3,2);

MAT=II*[L_p 0 L_r;0 M_q 0;N_p 0 N_r];
L_p=MAT(1,1);
L_r=MAT(1,3);
M_q=MAT(2,2);
N_p=MAT(3,1);
N_r=MAT(3,3);

MAT=II*[0,L_dA,L_dR,0;M_dE,0,0,M_dT;0,N_dA,N_dR,N_dT];
L_dA=MAT(1,2);
L_dR=MAT(1,3);
M_dE=MAT(2,1);
M_dT=MAT(2,4);
N_dA=MAT(3,2);
N_dR=MAT(3,3);
N_dT=MAT(3,4);


MAT=II*[0 0 0;0 0 M_wd;0 N_vd 0];
M_wd=MAT(2,3);
N_vd=MAT(3,2);




M_a=M_w*U0;
M_ad=M_wd*U0;
N_b=N_v*U0;
N_bd=N_vd*U0;
Y_b=Y_v*U0;
L_b=L_v*U0;
Z_a=Z_w*U0;