function perf=performance_calc(results,state,struc,engine,ref)

U0=state.AS;
% theta0=0;
% phi0=0;
alpha0=state.alpha;
beta0=state.betha;
g=state.g;
q=state.q;
c=ref.C_mac;
b=ref.b_ref;
s=ref.S_ref;



u0=U0;
w0=U0*alpha0;
v0=U0*beta0;

m=struc.mass_all;



CD0=results.CD0;
CD_u=0;
CD_a=results.CD_a;
CTX_u=0;
CL0=sum(results.CLwing);  %%%%??????
CL_u=0;
CL_a=results.CL_a;
CL_ad=0;
CL_q=results.CL_Q;


Cm0=results.Cm;
Cm_u=0;
Cm_a=results.Cm_a;
Cm_ad=0;
Cm_q=results.Cm_Q;
CmT_u=0;
CmT_a=0;



CD_dE=0;
CL_dE=0;
Cm_dE=0;
CD_iH=0;
CL_iH=0;
Cm_iH=0;
CmT0=0;

Cl_b=results.Cl_b;
Cl_p=results.Cl_P;
Cl_r=results.Cl_R;
CY_b=results.CY_b;
CY_p=results.CY_P;
CY_r=results.CY_R;
Cn_b=results.Cn_b;
CnT_b=0;
Cn_p=results.Cn_P;
Cn_r=results.Cn_R;

Cl_dA=0;
Cl_dR=0;
CY_dA=0;
CY_dR=0;
Cn_dA=-0;
Cn_dR=-0;

CTX0=0;

%%%%%%%%%%%%%%%%%%%%%%%%%%

I=struc.inertia_all;
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

perf.X_u=(q*s/m)*((2*u0/U0^2)*CX0+CX_u);
perf.XT_u=(q*s/m)*((2*u0/U0^2)*CTX0+CTX_u);
perf.X_w=(q*s/m)*((2*w0/U0^2)*CX0+CX_w);
perf.X_dE=(q*s/m)*CX_dE;
perf.X_q=0;
perf.X_dT=0;



perf.Z_u=(q*s/m)*((2*u0/U0^2)*CZ0+CZ_u);
perf.Z_w=(q*s/m)*((2*w0/U0^2)*CZ0+CZ_w);
perf.Z_wd=(q*s/m/2/U0)*c*CZ_w;  %%%
perf.Z_q=(q*s/m/2/U0)*c*CZ_q;
perf.Z_dE=(q*s/m)*CZ_dE;
perf.Z_dT=0;


perf.Y_v=(q*s/m)*CY_v;
perf.Y_p=(q*s/m/2/U0)*b*CY_p;
perf.Y_r=(q*s/m/2/U0)*b*CY_r;
perf.Y_dA=(q*s/m)*CY_dA;
perf.Y_dR=(q*s/m)*CY_dR;

perf.M_u=(q*s)*c*Cm_u;
perf.MT_u=(q*s)*c*CmT_u;
perf.M_w=(q*s)*c*Cm_w;
perf.MT_w=0;
perf.M_wd=(q*s/2/U0)*c^2*Cm_wd; %%
perf.M_q=(q*s/2/U0)*c^2*Cm_q;
perf.M_dE=(q*s)*c*Cm_dE;
perf.M_dT=0;
perf.M_theta=0;

perf.L_v=(q*s)*b*Cl_v;
perf.L_p=(q*s/2/U0)*b^2*Cl_p;
perf.L_r=(q*s/2/U0)*b^2*Cl_r;
perf.L_dA=(q*s)*b*Cl_dA;
perf.L_dR=(q*s)*b*Cl_dR;


perf.N_v=(q*s)*b*Cn_v;
perf.NT_v=0;
perf.N_vd=0;
perf.N_p=(q*s/2/U0)*b^2*Cn_p;
perf.N_r=(q*s/2/U0)*b^2*Cn_r;
perf.N_dA=(q*s)*b*Cn_dA;
perf.N_dR=(q*s)*b*Cn_dR;
perf.N_dT=0;




MAT=II*[0 perf.L_v 0;perf.M_u+perf.MT_u 0 perf.M_w+perf.MT_w;0 perf.N_v+perf.NT_v 0];
perf.L_v=MAT(1,2);
perf.M_u=MAT(2,1);
perf.M_w=MAT(2,3);
perf.N_v=MAT(3,2);

MAT=II*[perf.L_p 0 perf.L_r;0 perf.M_q 0;perf.N_p 0 perf.N_r];
perf.L_p=MAT(1,1);
perf.L_r=MAT(1,3);
perf.M_q=MAT(2,2);
perf.N_p=MAT(3,1);
perf.N_r=MAT(3,3);

MAT=II*[0,perf.L_dA,perf.L_dR,0;perf.M_dE,0,0,perf.M_dT;0,perf.N_dA,perf.N_dR,perf.N_dT];
perf.L_dA=MAT(1,2);
perf.L_dR=MAT(1,3);
perf.M_dE=MAT(2,1);
perf.M_dT=MAT(2,4);
perf.N_dA=MAT(3,2);
perf.N_dR=MAT(3,3);
perf.N_dT=MAT(3,4);


MAT=II*[0 0 0;0 0 perf.M_wd;0 perf.N_vd 0];
perf.M_wd=MAT(2,3);
perf.N_vd=MAT(3,2);




perf.M_a=perf.M_w*U0;
perf.M_ad=perf.M_wd*U0;
perf.N_b=perf.N_v*U0;
perf.N_bd=perf.N_vd*U0;
perf.Y_b=perf.Y_v*U0;
perf.L_b=perf.L_v*U0;
perf.Z_a=perf.Z_w*U0;




perf.stability.w_SP=sqrt(perf.Z_a*perf.M_q/U0-perf.M_a);
perf.stability.z_SP=-(perf.M_q+perf.Z_a/U0+perf.M_a)/(2*perf.stability.w_SP);
perf.stability.w_PH=sqrt(-g*perf.Z_u/U0);
perf.stability.z_PH=-(perf.X_u+perf.XT_u)/(2*perf.stability.w_PH);
perf.stability.w_D=sqrt(perf.N_b+(perf.Y_b*perf.N_r-perf.N_b*perf.Y_r)/U0);
perf.stability.z_D=-(perf.N_r+perf.Y_b/U0)/(2*perf.stability.w_D);
perf.stability.T_SR=(perf.L_b)/(perf.N_b*perf.L_r-perf.L_b*perf.N_r);
perf.stability.T_R=-1/perf.L_p;


perf.stability.SM=Cm_a/CL_a;

perf.LOD=results.CL/CD0;

if results.CL>0
    perf.ROC=sqrt(struc.mass_all/(0.5*state.rho*ref.S_ref*results.CL))*(engine.Thrust-results.D)/struc.mass_all;
else
    perf.ROC=0;
end
%
% perf.performance.Endurance=(results.CL/results.CD)*etta_prop/(engine.PSFC*state.AS)*log(struc.mass_all/struc.mass_all*0.8);
% perf.Range=(CL/CD)*etta_prop/BSFC*log(Wbegin/Wend);
% perf.ROC=sqrt(W/(0.5*rho*S*CL))*(T-D)/W;

MF=struc.mass_all/(struc.mass_all-0.8*struc.wf_max);
% perf.Endurance=perf.LOD/engine.TSFC*log(1/(1-MF));
perf.Endurance=perf.LOD/engine.TSFC*log(MF);
perf.Range=state.AS*perf.Endurance*3.6;

perf.n=results.L/(struc.mass_all*9.8);

%     w_SP_st=sqrt(-M_a);
%     z_SP_st=-(M_q+Z_a/U0+M_a)/(2*w_SP_st);

%     w_PH_st=g/U0*sqrt(2);
%     z_PH_st=sqrt(2)*(CD_u-CTX_u)/(4*CL0);



end