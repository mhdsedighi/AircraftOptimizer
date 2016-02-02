

i=1;

state=ans_state(i);
geo=ans_geo(i);
ref=ans_ref(i);



N_design=8;
N_morph=1;
N_trim=4;
N_act=1;
N_t=N_design+N_morph+N_trim+N_act;


for j=1:N_condition
    all_state(j)=state;
    all_struc(j)=struc;
end
 
all_struc(1).m_fuel=W_f*0.5;
all_struc(2).m_fuel=W_f*0.5;



con=1;
[R(con),lattice(con),perf(con),trim_cost(con),act(con),state(con),geo(con),struc(con),engine(con),ref(con)] = compute_aircraft3(design_code,morph_code(con,:),trim_code(con,:),actmass_code,state(con),geo(con),struc(con),body,act,all_state(con),all_ref(con));


