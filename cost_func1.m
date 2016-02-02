function cost=cost_func1(input,N_condition,N_design,N_morph,N_trim,N_act,all_state,geo,all_struc,body,act,engine,ref,finilize)


design_code(1:N_design)=input(1:N_design);
actmass_code(1:N_act)=input(N_design+1:N_design+N_act);

inputmat=vector2matrix(input(N_design+N_act+1:end),N_condition,N_trim+N_morph);

for j=1:N_condition
    morph_code(j,1:N_morph)=inputmat(j,1:N_morph);
    trim_code(j,1:N_trim)=inputmat(j,N_morph+1:N_morph+N_trim);
end



if finilize
    parfor j=1:N_condition
        [all_perf(j),all_trim_cost(j),all_act(j,:),all_state2(j),all_geo(j),all_struc2(j),all_engine(j),all_ref(j)] = compute_aircraft(design_code,morph_code(j,:),trim_code(j,:),actmass_code,all_state(j),geo,all_struc(j),body,act,engine,ref);
        %     x(j)=all_act;
    end
    
    assignin('base','ans_perf',all_perf);
    assignin('base','ans_trim_cost',all_trim_cost);
    assignin('base','ans_act',all_act);
    assignin('base','ans_state',all_state2);
    assignin('base','ans_geo',all_geo);
    assignin('base','ans_struc',all_struc2);
    assignin('base','ans_engine',all_engine);
    assignin('base','ans_ref',all_ref);
    
else
    parfor j=1:N_condition
        
        [all_perf(j),all_trim_cost(j),all_act(j,:),~,~,~,all_engine(j),~] = compute_aircraft(design_code,morph_code(j,:),trim_code(j,:),actmass_code,all_state(j),geo,all_struc(j),body,act,engine,ref);
        %     x(j)=all_act;
    end
end

mass_error=0;

for i=1:N_act
    for j=1:N_condition
        [all_act_out(j,i)]=actuator_sizing(all_act(j,i));
        mass_error=mass_error+abs(all_act(j,i).mass_guess-all_act_out(j,i).mass_guess);
    end
end

% [act_out]=actuator_sizing(act);

% cost=all_engine(1).PSFC*(sum(all_trim_cost)+mass_error);
cost=abs(all_engine(2).fuelFlow)*sum(all_trim_cost)+mass_error;


end

%
%

