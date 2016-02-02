
i=1;
%     assignin('base','ans_aero',all_aero);
%     assignin('base','ans_lattice',all_lattice);
%     assignin('base','ans_perf',all_perf);
%     assignin('base','ans_trim_cost',all_trim_cost);
%     assignin('base','ans_act',all_act);
%     assignin('base','ans_state',all_state2);
%     assignin('base','ans_geo',all_geo);
%     assignin('base','ans_struc',all_struc2);
%     assignin('base','ans_engine',all_engine);
%     assignin('base','ans_ref',all_ref);
%     assignin('base','ans_design_code',design_code);
%     assignin('base','ans_design_code',0);
%     assignin('base','ans_morph_code',0);
%     assignin('base','ans_trim_code',trim_code);
    
state=ans_state(i);
geo=ans_geo(i);
ref=ans_ref(i);
struc=ans_struc(i);
lattice=ans_lattice(i);


[CD0_wing,results.Re,results.Swet,results.Vol]=zeroliftdragpred(state.Mach,state.ALT,geo,ref);
%%% Compute blunt body drag increment
CD0_blunt=zldpblunt(state,body,ref);
%%% Summing up viscous drag
CD0=sum(sum(CD0_wing))+sum(CD0_blunt);
results.CD0=CD0;

engine=engine_calc(engine,state)
% engine.TSFC=engine.TSFC*6.5;

[results]=solver9(results,state,geo,lattice,ref);
[results]=coeff_create3(results,lattice,state,ref,geo);
perf=performance_calc(R,state,struc,engine,ref);

perf.Endurance


