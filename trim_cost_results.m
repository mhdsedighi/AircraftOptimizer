function [geo,state,results,struc,engine,ref ] = trim_cost_results(input,geo,state,results,struc,engine,ref)
%%% trim

alpha=input(1);
elevator_angle=input(2);
wing_angle=input(3);
throttle=input(4);
Xwing=input(5);
Zwing=input(6);


state.alpha=alpha;

engine.throttle=throttle;
engine=engine_calc(engine,state);


geo.startx(1)=geo.startx(1)+Xwing;
geo.startz(1)=geo.startz(1)+Zwing;

lattictype=1;
geo.flap_vector(2,1)=elevator_angle;
[lattice,ref]=fLattice_setup2(geo,state,lattictype,ref);

[struc]=massinertia(struc);

wingno=1;
Raxle=[0 1 0];
hinge_pos=squeeze(lattice.XYZ(1,1,:))';
[lattice]=wingrotation2(wingno,geo,lattice,Raxle,hinge_pos,wing_angle);
[results]=solver9(results,state,geo,lattice,ref);
[results]=coeff_create3(results,lattice,state,ref,geo);




weight_unitvector=[cos(state.phi)*sin(state.theta) sin(state.phi)*cos(state.alpha) -cos(state.phi)*cos(state.theta)];
weight_moment=cross((struc.cg_all'-geo.ref_point),struc.mass_all*state.g*weight_unitvector);

delta1=(results.CX-results.CD0)*state.q*ref.S_ref+engine.Thrust;
delta2=results.CZ*state.q*ref.S_ref-struc.mass_all*state.g;
delta3=results.Cm*state.q*ref.S_ref*ref.C_mac-engine.Thrust*(engine.pos(3)+geo.ref_point(3))+weight_moment(2);

% cost=delta1^2+delta2^2+delta3^2;
cost=abs(delta1)+abs(delta2)+abs(delta3);
% cost=abs(delta3);



end

