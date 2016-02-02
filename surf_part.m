function [mass,X_cg,Y_cg,Z_cg,I_XX,I_YY,I_ZZ,I_XY,I_YZ,I_XZ,S_wett]=surf_part(Xref,Yref,Zref,S,L,taper,sweep,dihedral,AOA_r,AOA_t,airfoil1,airfoil2,th,rho,sym)


% Xref=surf.x;
% Yref=surf.y;
% Zref=surf.z;
%
% S=surf.S;
% if surf.sym==1
%     S=S/2;
% end
%
% L=surf.L;
% taper=surf.taper;
% sweep=surf.sweep;
% dihedral=surf.dihedral;
% AOA=surf.AOA;
%
% rho=surf.rho;
% th=surf.th;
%
% Ixx=surf.airfoil.Ixx;
% Iyy=surf.airfoil.Iyy;
% Izz=surf.airfoil.Izz;
% Ixz=surf.airfoil.Ixz;
% x_cg=surf.airfoil.x_cg;
% z_cg=surf.airfoil.z_cg;
% s=surf.airfoil.s;
% p=surf.airfoil.p;
af1=strjoin(airfoil1);
af2=strjoin(airfoil2);
if af1==af2
    [x_cg,z_cg,p,s,Ixx,Iyy,Izz,Ixz] = airfoil_spec(af1);
else
    [x_cg1,z_cg1,p1,s1,Ixx1,Iyy1,Izz1,Ixz1] = airfoil_spec(af1);
    [x_cg2,z_cg2,p2,s2,Ixx2,Iyy2,Izz2,Ixz2] = airfoil_spec(af2);
    
    Ixx=mean([Ixx1 Ixx2]);
    Iyy=mean([Iyy1 Iyy2]);
    Izz=mean([Izz1 Izz2]);
    Ixz=mean([Ixz1 Ixz2]);
    x_cg=mean([x_cg1 x_cg2]);
    z_cg=mean([z_cg1 z_cg2]);
    s=mean([s1 s2]);
    p=mean([p1 p2]);
end

if sym==1
    S=S/2;
end

sweep=sweep*pi/180;
AOA=(AOA_r+AOA_t)*pi/180/2;

direction='right';

if (dihedral==90)
    dih=0;
    direction='right';
    
    Yref_old=Yref;
    Zref_old=Zref;
    
    Zref=Yref_old;
    Yref=-Zref_old;
    
elseif (dihedral==-90)||(dihedral==270)
    dih=0;
    direction='left';
    
    Yref_old=Yref;
    Zref_old=Zref;
    
    Zref=Yref_old;
    Yref=-Zref_old;
else
    dih=dihedral*pi/180;
end


if strcmp(th,'filled')
    
    V =(4*S^2*s*(taper^2 + taper + 1))/(3*L*(taper + 1)^2);
    
    
    X_cg =-(3*tan(sweep)*L^2*taper^2 + 2*tan(sweep)*L^2*taper + tan(sweep)*L^2 + 6*S*x_cg*taper^2 + 6*S*x_cg)/(4*L*(taper^2 + taper + 1));
    
    
    Y_cg =(L*(3*taper^2 + 2*taper + 1))/(4*(taper^2 + taper + 1));
    
    
    Z_cg =-(3*tan(dih)*L^2*taper^2 + 2*tan(dih)*L^2*taper + tan(dih)*L^2 + 6*S*z_cg*taper^2 + 6*S*z_cg)/(4*L*(taper^2 + taper + 1));
    
    
    I_XX =(S^3*(15*s*tan(dih)*L^2*taper^6*z_cg + 40*s*tan(dih)*L^2*taper^5*z_cg + 55*s*tan(dih)*L^2*taper^4*z_cg + 60*s*tan(dih)*L^2*taper^3*z_cg + 45*s*tan(dih)*L^2*taper^2*z_cg + 20*s*tan(dih)*L^2*taper*z_cg + 5*s*tan(dih)*L^2*z_cg + 15*S*s*taper^6*z_cg^2 + 16*Ixx*S*taper^6 + 30*S*s*taper^5*z_cg^2 + 32*Ixx*S*taper^5 + 45*S*s*taper^4*z_cg^2 + 48*Ixx*S*taper^4 + 60*S*s*taper^3*z_cg^2 + 48*Ixx*S*taper^3 + 45*S*s*taper^2*z_cg^2 + 48*Ixx*S*taper^2 + 30*S*s*taper*z_cg^2 + 32*Ixx*S*taper + 15*S*s*z_cg^2 + 16*Ixx*S))/(5*L^3*(taper + 1)^4*(taper^2 + taper + 1)) + (L*S^2*s*(3*taper^3 + 5*taper^2 + 3*taper + 1)^2)/(6*(cos(2*dih) + 1)*(taper + 1)^4*(taper^2 + taper + 1));
    
    
    I_YY =(S^3*(15*s*tan(sweep)*L^2*taper^6*x_cg + 15*s*tan(dih)*L^2*taper^6*z_cg + 40*s*tan(sweep)*L^2*taper^5*x_cg + 40*s*tan(dih)*L^2*taper^5*z_cg + 55*s*tan(sweep)*L^2*taper^4*x_cg + 55*s*tan(dih)*L^2*taper^4*z_cg + 60*s*tan(sweep)*L^2*taper^3*x_cg + 60*s*tan(dih)*L^2*taper^3*z_cg + 45*s*tan(sweep)*L^2*taper^2*x_cg + 45*s*tan(dih)*L^2*taper^2*z_cg + 20*s*tan(sweep)*L^2*taper*x_cg + 20*s*tan(dih)*L^2*taper*z_cg + 5*s*tan(sweep)*L^2*x_cg + 5*s*tan(dih)*L^2*z_cg + 15*S*s*taper^6*x_cg^2 + 15*S*s*taper^6*z_cg^2 + 16*Iyy*S*taper^6 + 30*S*s*taper^5*x_cg^2 + 30*S*s*taper^5*z_cg^2 + 32*Iyy*S*taper^5 + 45*S*s*taper^4*x_cg^2 + 45*S*s*taper^4*z_cg^2 + 48*Iyy*S*taper^4 + 60*S*s*taper^3*x_cg^2 + 60*S*s*taper^3*z_cg^2 + 48*Iyy*S*taper^3 + 45*S*s*taper^2*x_cg^2 + 45*S*s*taper^2*z_cg^2 + 48*Iyy*S*taper^2 + 30*S*s*taper*x_cg^2 + 30*S*s*taper*z_cg^2 + 32*Iyy*S*taper + 15*S*s*x_cg^2 + 15*S*s*z_cg^2 + 16*Iyy*S))/(5*L^3*(taper + 1)^4*(taper^2 + taper + 1)) + (L*S^2*s*(tan(dih)^2 + tan(sweep)^2)*(3*taper^2 + 2*taper + 1)^2)/(12*(taper + 1)^2*(taper^2 + taper + 1));
    
    
    I_ZZ =(S^3*(15*s*tan(sweep)*L^2*taper^6*x_cg + 40*s*tan(sweep)*L^2*taper^5*x_cg + 55*s*tan(sweep)*L^2*taper^4*x_cg + 60*s*tan(sweep)*L^2*taper^3*x_cg + 45*s*tan(sweep)*L^2*taper^2*x_cg + 20*s*tan(sweep)*L^2*taper*x_cg + 5*s*tan(sweep)*L^2*x_cg + 15*S*s*taper^6*x_cg^2 + 16*Izz*S*taper^6 + 30*S*s*taper^5*x_cg^2 + 32*Izz*S*taper^5 + 45*S*s*taper^4*x_cg^2 + 48*Izz*S*taper^4 + 60*S*s*taper^3*x_cg^2 + 48*Izz*S*taper^3 + 45*S*s*taper^2*x_cg^2 + 48*Izz*S*taper^2 + 30*S*s*taper*x_cg^2 + 32*Izz*S*taper + 15*S*s*x_cg^2 + 16*Izz*S))/(5*L^3*(taper + 1)^4*(taper^2 + taper + 1)) + (L*S^2*s*(3*taper^3 + 5*taper^2 + 3*taper + 1)^2)/(6*(cos(2*sweep) + 1)*(taper + 1)^4*(taper^2 + taper + 1));
    
    
    I_XY =-(S^2*s*(3*taper^2 + 2*taper + 1)*(3*tan(sweep)*L^2*taper^2 + 2*tan(sweep)*L^2*taper + tan(sweep)*L^2 + 6*S*x_cg*taper^2 + 6*S*x_cg))/(12*L*(taper + 1)^2*(taper^2 + taper + 1));
    
    
    I_XZ =((S^4*(16*Ixz + 32*Ixz*taper + 48*Ixz*taper^2 + 48*Ixz*taper^3 + 48*Ixz*taper^4 + 32*Ixz*taper^5 + 16*Ixz*taper^6 + 15*s*x_cg*z_cg + 30*s*taper*x_cg*z_cg + 45*s*taper^2*x_cg*z_cg + 60*s*taper^3*x_cg*z_cg + 45*s*taper^4*x_cg*z_cg + 30*s*taper^5*x_cg*z_cg + 15*s*taper^6*x_cg*z_cg))/5 + (L^2*S^3*s*(x_cg*tan(dih) + z_cg*tan(sweep))*(taper + 1)^2*(3*taper^4 + 2*taper^3 + 4*taper^2 + 2*taper + 1))/2)/(L^3*(taper + 1)^4*(taper^2 + taper + 1)) + (L*S^2*s*tan(dih)*tan(sweep)*(3*taper^2 + 2*taper + 1)^2)/(12*(taper + 1)^2*(taper^2 + taper + 1));
    
    
    I_YZ =-(S^2*s*(3*taper^2 + 2*taper + 1)*(3*tan(dih)*L^2*taper^2 + 2*tan(dih)*L^2*taper + tan(dih)*L^2 + 6*S*z_cg*taper^2 + 6*S*z_cg))/(12*L*(taper + 1)^2*(taper^2 + taper + 1));
    
    
    S_wett =S*p;
    
    
else
    
    x_cg =(s*x_cg - (s - s*(2*th - 1)^2)*(th + x_cg - x_cg*(2*th - 1)))/(s*(2*th - 1)^2);
    
    
    V =4*s*th*(S - L*th);
    
    
    X_cg =-(2*S^2*taper + 8*S^2*x_cg + 2*S^2 - 4*S^2*taper^2 + 2*L^2*S*tan(sweep) + 8*S^2*taper^2*x_cg - 3*L^3*th*tan(sweep) - 3*L*S*th + 8*S^2*taper*x_cg + 4*L^2*S*taper^2*tan(sweep) - 3*L^3*taper^2*th*tan(sweep) + 3*L*S*taper^2*th + 6*L^2*S*taper*tan(sweep) - 6*L^3*taper*th*tan(sweep) - 6*L*S*th*x_cg - 12*L*S*taper*th*x_cg - 6*L*S*taper^2*th*x_cg)/(6*L*(S - L*th)*(taper + 1)^2);
    
    
    Y_cg =(L*(2*S - 3*L*th + 4*S*taper - 3*L*taper*th))/(6*(S - L*th)*(taper + 1));
    
    
    Z_cg =-(12*S*z_cg - 3*S*sin(AOA) - 6*L^2*th*tan(dih) - 12*L*th*z_cg + 12*S*taper*z_cg + 3*L*th*sin(AOA) + 4*L*S*tan(dih) - 3*S*taper*sin(AOA) + 12*S*x_cg*sin(AOA) - 12*L*taper*th*z_cg + 3*L*taper*th*sin(AOA) + 8*L*S*taper*tan(dih) - 12*L*th*x_cg*sin(AOA) + 12*S*taper*x_cg*sin(AOA) - 6*L^2*taper*th*tan(dih) - 12*L*taper*th*x_cg*sin(AOA))/(12*(S - L*th)*(taper + 1));
    
    
    I_XX =L^2*((32*Ixx*S^4*(taper - 1))/(L^5*(taper + 1)^4) + (32*Ixx*S*(taper - 1)*(L*th - S + L*taper*th)^3)/(L^5*(taper + 1)^4) + (4*S*s*th*(taper - 1)*((12*S*z_cg - 3*S*sin(AOA) - 6*L^2*th*tan(dih) - 12*L*th*z_cg + 12*S*taper*z_cg + 3*L*th*sin(AOA) + 4*L*S*tan(dih) - 3*S*taper*sin(AOA) + 12*S*x_cg*sin(AOA) - 12*L*taper*th*z_cg + 3*L*taper*th*sin(AOA) + 8*L*S*taper*tan(dih) - 12*L*th*x_cg*sin(AOA) + 12*S*taper*x_cg*sin(AOA) - 6*L^2*taper*th*tan(dih) - 12*L*taper*th*x_cg*sin(AOA))^2/(144*(S - L*th)^2*(taper + 1)^2) + (L^2*(2*S - 3*L*th + 4*S*taper - 3*L*taper*th)^2)/(36*(S - L*th)^2*(taper + 1)^2)))/(L^2*(taper + 1)) + (16*Ixx*S^2*th*(taper - 1)^2*(3*S - 2*L*th + S*taper - 2*L*taper*th))/(L^4*(taper + 1)^3)) + (16*Ixx*S^4)/(L^3*(taper + 1)^4) - (16*Ixx*(L*th - S + L*taper*th)^4)/(L^3*(taper + 1)^4) - (4*s*th*((12*S*z_cg - 3*S*sin(AOA) - 6*L^2*th*tan(dih) - 12*L*th*z_cg + 12*S*taper*z_cg + 3*L*th*sin(AOA) + 4*L*S*tan(dih) - 3*S*taper*sin(AOA) + 12*S*x_cg*sin(AOA) - 12*L*taper*th*z_cg + 3*L*taper*th*sin(AOA) + 8*L*S*taper*tan(dih) - 12*L*th*x_cg*sin(AOA) + 12*S*taper*x_cg*sin(AOA) - 6*L^2*taper*th*tan(dih) - 12*L*taper*th*x_cg*sin(AOA))^2/(144*(S - L*th)^2*(taper + 1)^2) + (L^2*(2*S - 3*L*th + 4*S*taper - 3*L*taper*th)^2)/(36*(S - L*th)^2*(taper + 1)^2))*(L*th - 2*S + L*taper*th))/(taper + 1);
    
    
    I_YY =L^2*((32*Iyy*S^4*(taper - 1))/(L^5*(taper + 1)^4) + (32*Iyy*S*(taper - 1)*(L*th - S + L*taper*th)^3)/(L^5*(taper + 1)^4) + (4*S*s*th*((12*S*z_cg - 3*S*sin(AOA) - 6*L^2*th*tan(dih) - 12*L*th*z_cg + 12*S*taper*z_cg + 3*L*th*sin(AOA) + 4*L*S*tan(dih) - 3*S*taper*sin(AOA) + 12*S*x_cg*sin(AOA) - 12*L*taper*th*z_cg + 3*L*taper*th*sin(AOA) + 8*L*S*taper*tan(dih) - 12*L*th*x_cg*sin(AOA) + 12*S*taper*x_cg*sin(AOA) - 6*L^2*taper*th*tan(dih) - 12*L*taper*th*x_cg*sin(AOA))^2/(144*(S - L*th)^2*(taper + 1)^2) + (2*S^2*taper + 8*S^2*x_cg + 2*S^2 - 4*S^2*taper^2 + 2*L^2*S*tan(sweep) + 8*S^2*taper^2*x_cg - 3*L^3*th*tan(sweep) - 3*L*S*th + 8*S^2*taper*x_cg + 4*L^2*S*taper^2*tan(sweep) - 3*L^3*taper^2*th*tan(sweep) + 3*L*S*taper^2*th + 6*L^2*S*taper*tan(sweep) - 6*L^3*taper*th*tan(sweep) - 6*L*S*th*x_cg - 12*L*S*taper*th*x_cg - 6*L*S*taper^2*th*x_cg)^2/(36*L^2*(S - L*th)^2*(taper + 1)^4))*(taper - 1))/(L^2*(taper + 1)) + (16*Iyy*S^2*th*(taper - 1)^2*(3*S - 2*L*th + S*taper - 2*L*taper*th))/(L^4*(taper + 1)^3)) + (16*Iyy*S^4)/(L^3*(taper + 1)^4) - (16*Iyy*(L*th - S + L*taper*th)^4)/(L^3*(taper + 1)^4) - (4*s*th*((12*S*z_cg - 3*S*sin(AOA) - 6*L^2*th*tan(dih) - 12*L*th*z_cg + 12*S*taper*z_cg + 3*L*th*sin(AOA) + 4*L*S*tan(dih) - 3*S*taper*sin(AOA) + 12*S*x_cg*sin(AOA) - 12*L*taper*th*z_cg + 3*L*taper*th*sin(AOA) + 8*L*S*taper*tan(dih) - 12*L*th*x_cg*sin(AOA) + 12*S*taper*x_cg*sin(AOA) - 6*L^2*taper*th*tan(dih) - 12*L*taper*th*x_cg*sin(AOA))^2/(144*(S - L*th)^2*(taper + 1)^2) + (2*S^2*taper + 8*S^2*x_cg + 2*S^2 - 4*S^2*taper^2 + 2*L^2*S*tan(sweep) + 8*S^2*taper^2*x_cg - 3*L^3*th*tan(sweep) - 3*L*S*th + 8*S^2*taper*x_cg + 4*L^2*S*taper^2*tan(sweep) - 3*L^3*taper^2*th*tan(sweep) + 3*L*S*taper^2*th + 6*L^2*S*taper*tan(sweep) - 6*L^3*taper*th*tan(sweep) - 6*L*S*th*x_cg - 12*L*S*taper*th*x_cg - 6*L*S*taper^2*th*x_cg)^2/(36*L^2*(S - L*th)^2*(taper + 1)^4))*(L*th - 2*S + L*taper*th))/(taper + 1);
    
    
    I_ZZ =L^2*((32*Izz*S^4*(taper - 1))/(L^5*(taper + 1)^4) + (32*Izz*S*(taper - 1)*(L*th - S + L*taper*th)^3)/(L^5*(taper + 1)^4) + (4*S*s*th*((L^2*(2*S - 3*L*th + 4*S*taper - 3*L*taper*th)^2)/(36*(S - L*th)^2*(taper + 1)^2) + (2*S^2*taper + 8*S^2*x_cg + 2*S^2 - 4*S^2*taper^2 + 2*L^2*S*tan(sweep) + 8*S^2*taper^2*x_cg - 3*L^3*th*tan(sweep) - 3*L*S*th + 8*S^2*taper*x_cg + 4*L^2*S*taper^2*tan(sweep) - 3*L^3*taper^2*th*tan(sweep) + 3*L*S*taper^2*th + 6*L^2*S*taper*tan(sweep) - 6*L^3*taper*th*tan(sweep) - 6*L*S*th*x_cg - 12*L*S*taper*th*x_cg - 6*L*S*taper^2*th*x_cg)^2/(36*L^2*(S - L*th)^2*(taper + 1)^4))*(taper - 1))/(L^2*(taper + 1)) + (16*Izz*S^2*th*(taper - 1)^2*(3*S - 2*L*th + S*taper - 2*L*taper*th))/(L^4*(taper + 1)^3)) + (16*Izz*S^4)/(L^3*(taper + 1)^4) - (16*Izz*(L*th - S + L*taper*th)^4)/(L^3*(taper + 1)^4) - (4*s*th*((L^2*(2*S - 3*L*th + 4*S*taper - 3*L*taper*th)^2)/(36*(S - L*th)^2*(taper + 1)^2) + (2*S^2*taper + 8*S^2*x_cg + 2*S^2 - 4*S^2*taper^2 + 2*L^2*S*tan(sweep) + 8*S^2*taper^2*x_cg - 3*L^3*th*tan(sweep) - 3*L*S*th + 8*S^2*taper*x_cg + 4*L^2*S*taper^2*tan(sweep) - 3*L^3*taper^2*th*tan(sweep) + 3*L*S*taper^2*th + 6*L^2*S*taper*tan(sweep) - 6*L^3*taper*th*tan(sweep) - 6*L*S*th*x_cg - 12*L*S*taper*th*x_cg - 6*L*S*taper^2*th*x_cg)^2/(36*L^2*(S - L*th)^2*(taper + 1)^4))*(L*th - 2*S + L*taper*th))/(taper + 1);
    
    
    I_XY =(s*th*(2*S - 3*L*th + 4*S*taper - 3*L*taper*th)^2*(2*S^2*taper + 8*S^2*x_cg + 2*S^2 - 4*S^2*taper^2 + 2*L^2*S*tan(sweep) + 8*S^2*taper^2*x_cg - 3*L^3*th*tan(sweep) - 3*L*S*th + 8*S^2*taper*x_cg + 4*L^2*S*taper^2*tan(sweep) - 3*L^3*taper^2*th*tan(sweep) + 3*L*S*taper^2*th + 6*L^2*S*taper*tan(sweep) - 6*L^3*taper*th*tan(sweep) - 6*L*S*th*x_cg - 12*L*S*taper*th*x_cg - 6*L*S*taper^2*th*x_cg)^2)/(324*(S - L*th)^3*(taper + 1)^6);
    
    
    I_XZ =L^2*((32*Ixz*S^4*(taper - 1))/(L^5*(taper + 1)^4) + (32*Ixz*S*(taper - 1)*(L*th - S + L*taper*th)^3)/(L^5*(taper + 1)^4) + (16*Ixz*S^2*th*(taper - 1)^2*(3*S - 2*L*th + S*taper - 2*L*taper*th))/(L^4*(taper + 1)^3) + (S*s*th*(taper - 1)*(12*S*z_cg - 3*S*sin(AOA) - 6*L^2*th*tan(dih) - 12*L*th*z_cg + 12*S*taper*z_cg + 3*L*th*sin(AOA) + 4*L*S*tan(dih) - 3*S*taper*sin(AOA) + 12*S*x_cg*sin(AOA) - 12*L*taper*th*z_cg + 3*L*taper*th*sin(AOA) + 8*L*S*taper*tan(dih) - 12*L*th*x_cg*sin(AOA) + 12*S*taper*x_cg*sin(AOA) - 6*L^2*taper*th*tan(dih) - 12*L*taper*th*x_cg*sin(AOA))^2*(2*S^2*taper + 8*S^2*x_cg + 2*S^2 - 4*S^2*taper^2 + 2*L^2*S*tan(sweep) + 8*S^2*taper^2*x_cg - 3*L^3*th*tan(sweep) - 3*L*S*th + 8*S^2*taper*x_cg + 4*L^2*S*taper^2*tan(sweep) - 3*L^3*taper^2*th*tan(sweep) + 3*L*S*taper^2*th + 6*L^2*S*taper*tan(sweep) - 6*L^3*taper*th*tan(sweep) - 6*L*S*th*x_cg - 12*L*S*taper*th*x_cg - 6*L*S*taper^2*th*x_cg)^2)/(1296*L^4*(S - L*th)^4*(taper + 1)^7)) + (16*Ixz*S^4)/(L^3*(taper + 1)^4) - (16*Ixz*(L*th - S + L*taper*th)^4)/(L^3*(taper + 1)^4) - (s*th*(L*th - 2*S + L*taper*th)*(12*S*z_cg - 3*S*sin(AOA) - 6*L^2*th*tan(dih) - 12*L*th*z_cg + 12*S*taper*z_cg + 3*L*th*sin(AOA) + 4*L*S*tan(dih) - 3*S*taper*sin(AOA) + 12*S*x_cg*sin(AOA) - 12*L*taper*th*z_cg + 3*L*taper*th*sin(AOA) + 8*L*S*taper*tan(dih) - 12*L*th*x_cg*sin(AOA) + 12*S*taper*x_cg*sin(AOA) - 6*L^2*taper*th*tan(dih) - 12*L*taper*th*x_cg*sin(AOA))^2*(2*S^2*taper + 8*S^2*x_cg + 2*S^2 - 4*S^2*taper^2 + 2*L^2*S*tan(sweep) + 8*S^2*taper^2*x_cg - 3*L^3*th*tan(sweep) - 3*L*S*th + 8*S^2*taper*x_cg + 4*L^2*S*taper^2*tan(sweep) - 3*L^3*taper^2*th*tan(sweep) + 3*L*S*taper^2*th + 6*L^2*S*taper*tan(sweep) - 6*L^3*taper*th*tan(sweep) - 6*L*S*th*x_cg - 12*L*S*taper*th*x_cg - 6*L*S*taper^2*th*x_cg)^2)/(1296*L^2*(S - L*th)^4*(taper + 1)^7);
    
    
    I_YZ =(L^2*s*th*(2*S - 3*L*th + 4*S*taper - 3*L*taper*th)^2*(12*S*z_cg - 3*S*sin(AOA) - 6*L^2*th*tan(dih) - 12*L*th*z_cg + 12*S*taper*z_cg + 3*L*th*sin(AOA) + 4*L*S*tan(dih) - 3*S*taper*sin(AOA) + 12*S*x_cg*sin(AOA) - 12*L*taper*th*z_cg + 3*L*taper*th*sin(AOA) + 8*L*S*taper*tan(dih) - 12*L*th*x_cg*sin(AOA) + 12*S*taper*x_cg*sin(AOA) - 6*L^2*taper*th*tan(dih) - 12*L*taper*th*x_cg*sin(AOA))^2)/(1296*(S - L*th)^3*(taper + 1)^4);
    
    
    S_wett =S*p;
    
    
end

% X_cg=X_cg+Xref;
% Z_cg=Z_cg+Zref;

if strcmp(direction,'right')
    %     Y_cg=Y_cg+Yref;
    I_XY=(I_XY-V*(X_cg*Y_cg))*rho;
    I_YZ=(I_YZ-V*(Y_cg*Z_cg))*rho;
elseif strcmp(direction,'left')
    %     Y_cg=Y_cg-Yref;
    I_XY=(-I_XY-V*(X_cg*Y_cg))*rho;
    I_YZ=(-I_YZ-V*(Y_cg*Z_cg))*rho;
end

mass=V*rho;
I_XX=(I_XX-V*(Y_cg^2+Z_cg^2))*rho;
I_YY=(I_YY-V*(X_cg^2+Z_cg^2))*rho;
I_ZZ=(I_ZZ-V*(X_cg^2+Y_cg^2))*rho;
I_XZ=(I_XZ-V*(X_cg*Z_cg))*rho;



%
X_cg=X_cg+Xref;
Z_cg=Z_cg+Zref;

if strcmp(direction,'right')
    Y_cg=Y_cg+Yref;
elseif strcmp(direction,'left')
    Y_cg=Y_cg-Yref;
end
%


if (dihedral==90)||(dihedral==-90)||(dihedral==270)
    
    Y_cg_old=Y_cg;
    Z_cg_old=Z_cg;
    I_YY_old=I_YY;
    I_ZZ_old=I_ZZ;
    I_XZ_old=I_XZ;
    I_XY_old=I_XY;
    I_YZ_old=I_YZ;
    
    
    Y_cg=Z_cg_old;
    Z_cg=-Y_cg_old;
    I_YY=I_ZZ_old;
    I_ZZ=I_YY_old;
    I_XZ=-I_XY_old;
    I_XY=-I_XZ_old;
    I_YZ=-I_YZ_old;
end


if sym==1
    Y_cg=0;
    I_XY=0;
    I_YZ=0;
    I_XX=2*I_XX;
    I_YY=2*I_YY;
    I_ZZ=2*I_ZZ;
    I_XZ=2*I_XZ;
    S_wett=2*S_wett;
    mass=2*mass;
end




% part.mass=mass;
% part.X_cg=X_cg;
% part.Y_cg=Y_cg;
% part.Z_cg=Z_cg;
% part.I_XX=I_XX;
% part.I_YY=I_YY;
% part.I_ZZ=I_ZZ;
% part.I_XY=I_XY;
% part.I_YZ=I_YZ;
% part.I_XZ=I_XZ;
% part.S_wett=S_wett;



end

