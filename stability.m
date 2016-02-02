
w_SP=sqrt(Z_a*M_q/U0-M_a);
z_SP=-(M_q+Z_a/U0+M_a)/(2*w_SP);

%     w_SP_st=sqrt(-M_a);
%     z_SP_st=-(M_q+Z_a/U0+M_a)/(2*w_SP_st);

w_PH=sqrt(-g*Z_u/U0);
z_PH=-(X_u+XT_u)/(2*w_PH);

%     w_PH_st=g/U0*sqrt(2);
%     z_PH_st=sqrt(2)*(CD_u-CTX_u)/(4*CL0);zzzzz


w_D=sqrt(N_b+(Y_b*N_r-N_b*Y_r)/U0);
z_D=-(N_r+Y_b/U0)/(2*w_D);
T_SR=(L_b)/(N_b*L_r-L_b*N_r);
T_R=-1/L_p;

