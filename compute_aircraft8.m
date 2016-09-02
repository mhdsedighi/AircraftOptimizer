function [results,perf,trim_cost,act,state,geo,struc,engine,ref] = compute_aircraft8(design_code,morph_code,trim_code,actmass_code,state,geo,struc,body,act,engine,ref)


lattictype=1;
wingno=1;
b=54.76*0.3048;

Xwing=design_code(1);
Zwing=design_code(2);
bfrac1=design_code(3);
% bfrac2=design_code(4);
sweep=design_code(4);
twist=design_code(5);
% taper=design_code(6);

% Xwing=design_code(1);
% Zwing=design_code(2);
% twist=design_code(3);

alpha=trim_code(1);
elevator_angle=trim_code(2);
throttle=trim_code(3);
AS=trim_code(4)*100;

geo.startx(1)=geo.startx(1)+Xwing;
geo.startz(1)=geo.startz(1)+Zwing;
% geo.b(1,:)=[b*(1-bfrac1-bfrac2) b*bfrac1 b*bfrac2];
geo.b(1,:)=[b*(1-bfrac1)/2 b*(1-bfrac1)/2 b*bfrac1];
geo.SW(1,:)=[sweep sweep sweep];
% geo.T(1,3)=taper;

state.alpha=alpha;
state.AS=AS;
engine.throttle=throttle;
geo.flap_vector(2,1)=elevator_angle;

act(1).mass_guess=actmass_code(1)*100;
% act(2).mass_guess=actmass_code(2);



% nsec=geo.nelem(3);
% for j=1:nsec
%     geo.TW(1,j,1)=geo.twist(1,j);
% end
% for j=1:nsec-1
%     geo.TW(1,j,2)=geo.twist(1,j+1);
% end


struc.pointmass=act;
[struc]=massinertia(struc);

engine=engine_calc(engine,state);

[CD0_wing,results.Re,results.Swet,results.Vol]=zeroliftdragpred(state.Mach,state.ALT,geo,ref);
CD0_blunt=zldpblunt(state,body,ref);
CD0=sum(sum(CD0_wing))+sum(CD0_blunt);
results.CD0=CD0;

[lattice,ref]=fLattice_setup2(geo,state,lattictype,ref);
Raxle=[0 1 0];
hinge_pos=squeeze(lattice.XYZ(1,1,:))';
[lattice]=wingrotation2(wingno,geo,lattice,Raxle,hinge_pos,twist);



% 
% partno=3;
% act_num=1;
% course_portion=morph_code(1);
% [geo,lattice,struc,act]=telescoping(act_num,geo,ref,lattice,results,state,struc,act,wingno,partno,course_portion);


partno=3;
act_num=1;
rot_angle=morph_code(1);
Raxle_local=morph_code(2:4);
hinge_portion=0.5;
[geo,lattice,struc,act]=part_rotation(act_num,geo,lattice,results,struc,state,act,wingno,partno,Raxle_local,rot_angle,hinge_portion,ref);




[results]=solver9(results,state,geo,lattice,ref);
[results]=coeff_create3(results,lattice,state,ref,geo);
perf=performance_calc(results,state,struc,engine,ref);

%%%trim
% weight_unitvector=[cos(state.phi)*sin(state.theta) sin(state.phi)*cos(state.alpha) -cos(state.phi)*cos(state.theta)];
% weight_moment=cross((struc.cg_all'-geo.ref_point),struc.mass_all*state.g*weight_unitvector);

[rho,a,~,~]=ISAtmosphere(state.ALT);     %Calling International Standard atmosphere.
state.rho=rho;
state.Mach=state.AS/a;
state.q=0.5*(state.rho)*(state.AS)^2;


ca=cos(state.alpha);
sa=sin(state.alpha);
B2WTransform=[ca,0,sa ;0,1,0 ;-sa,0,ca];

Thrust_vec_b=engine.Thrust*[1;0;0];
Thrust_vec_w=B2WTransform*Thrust_vec_b;
weight_vec_w=struc.mass_all*state.g*[0;0;-1];
weight_vec_b=B2WTransform'*weight_vec_w;

weight_moment=cross((struc.cg_all'-geo.ref_point),weight_vec_b);

delta1=-(results.CD)*state.q*ref.S_ref+Thrust_vec_w(1)+weight_vec_w(1);
delta2=results.CL*state.q*ref.S_ref+Thrust_vec_w(3)+weight_vec_w(3);
delta3=results.Cm*state.q*ref.S_ref*ref.C_mac+weight_moment(2);

trim_cost=abs(delta1)+abs(delta2)+abs(delta3);


end

