function cost=cost_func7(input,N_condition,N_design,N_morph,N_trim,N_act,all_state,geo,all_struc,body,act,engine,ref,finilize)


design_code(1:N_design)=input(1:N_design);
% actmass_code(1:N_act)=input(N_design+1:N_design+N_act);

inputmat=vector2matrix(input(N_design+N_act+1:end),N_condition,N_trim+N_morph);

for j=1:N_condition
%     morph_code(j,1:N_morph)=inputmat(j,1:N_morph);
    trim_code(j,1:N_trim)=inputmat(j,N_morph+1:N_morph+N_trim);
end



if finilize
    for j=1:N_condition
        [all_aero(j),all_lattice(j),all_perf(j),all_trim_cost(j),all_act(j,:),all_state2(j),all_geo(j),all_struc2(j),all_engine(j),all_ref(j)] = compute_aircraft2(design_code,trim_code(j,:),all_state(j),geo,all_struc(j),body,act,engine,ref);
        %     x(j)=all_act;
    end
    assignin('base','ans_aero',all_aero);
    assignin('base','ans_lattice',all_lattice);
    assignin('base','ans_perf',all_perf);
    assignin('base','ans_trim_cost',all_trim_cost);
    assignin('base','ans_act',all_act);
    assignin('base','ans_state',all_state2);
    assignin('base','ans_geo',all_geo);
    assignin('base','ans_struc',all_struc2);
    assignin('base','ans_engine',all_engine);
    assignin('base','ans_ref',all_ref);
    assignin('base','ans_design_code',design_code);
    assignin('base','ans_act_code',0);
    assignin('base','ans_morph_code',0);
    assignin('base','ans_trim_code',trim_code);
    
else
    for j=1:N_condition
        
        [~,~,all_perf(j),all_trim_cost(j),all_act(j,:),~,~,~,all_engine(j),~] =  compute_aircraft2(design_code,trim_code(j,:),all_state(j),geo,all_struc(j),body,act,engine,ref);
        %     x(j)=all_act;
    end
end


% for i=1:N_act
%     for j=1:N_condition
%         [all_act_out(j,i)]=actuator_sizing(all_act(j,i));
%         mass_error=mass_error+abs(all_act(j,i).mass_guess-all_act_out(j,i).mass_guess);
%     end
% end



% [act_out]=actuator_sizing(act);

% cost=all_engine(1).PSFC*(sum(all_trim_cost)+mass_error);
% s1=abs(all_state(1).AS/(all_engine(1).fuelFlow)+1)+abs(all_state(2).AS/(all_engine(2).fuelFlow+1))+abs(all_state(3).AS/(all_engine(3).fuelFlow)+1);
% s2=all_perf(4).ROC;
% s3=all_state(2).AS;
% 
% s4=sum(all_trim_cost)+mass_error;
% 
% cost=-s1-s2-s3+s4;

cost_a=1;
cost_st=0;
for i=1:N_condition
    %     cost_st=cost_st+(all_perf(i).stability.SM+0.15).^2;
    if all_perf(i).stability.SM>-0.01
        cost_st=cost_st+abs(all_perf(i).stability.SM);
    end
     if all_perf(i).LOD<0
        cost_a=cost_a+2;
     end
     if all_perf(i).ROC<0
        cost_a=cost_a+2;
     end 
     if  abs(all_perf(i).n)>2
        cost_a=cost_a+2;
     end
    
%     cost_perf1=cost_perf1+1/all_engine(i).fuelFlow;
end


cost1=sum(all_trim_cost);
% cost2=(all_perf(:).stability.SM+0.15)^2;
cost_v=1/all_state(1).AS;

% cost=sum(all_trim_cost);
cost=(cost_v)*(cost1*cost_a*(cost_st+1))^2;

% perf_cost1=abs(all_state(1).AS/all_engine(1).fuelFlow);
% perf_cost2=all_state(4).AS;
% perf_cost3=abs(all_perf(5).ROC);
% 
% cost=1/(perf_cost1+perf_cost2+perf_cost3)*(sum(all_trim_cost));

% cost=1/(perf_cost1+1)+(sum(all_trim_cost));


end

%
%

