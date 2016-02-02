function cost=cost_func5(input,N_condition,N_design,N_morph,N_trim,N_act,all_state,geo,all_struc,body,act,engine,ref,finilize)


design_code(1:N_design)=input(1:N_design);
actmass_code(1:N_act)=input(N_design+1:N_design+N_act);

inputmat=vector2matrix(input(N_design+N_act+1:end),N_condition,N_trim+N_morph);

for k=1:N_condition
    morph_code(k,1:N_morph)=inputmat(k,1:N_morph);
    trim_code(k,1:N_trim)=inputmat(k,N_morph+1:N_morph+N_trim);
end


if finilize
    for k=1:N_condition
        [aero,all_lattice(k),all_perf(k),all_trim_cost(k),all_act(k,:),all_state2(k),all_geo(k),all_struc2(k),all_engine(k),all_ref(k)] = compute_aircraft3(design_code,morph_code(k,:),trim_code(k,:),actmass_code,all_state(k),geo,all_struc(k),body,act,engine,ref);
        aero.sonicCP=0;
        aero.sonicWarning=0;
        aero.sonicFraction=0;
        all_aero(k)=aero;
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
    assignin('base','ans_act_code',actmass_code);
    assignin('base','ans_morph_code',morph_code);
    assignin('base','ans_trim_code',trim_code);
    
else
    parfor k=1:N_condition
        
        [~,~,all_perf(k),all_trim_cost(k),all_act(k,:),~,~,~,all_engine(k),~] = compute_aircraft3(design_code,morph_code(k,:),trim_code(k,:),actmass_code,all_state(k),geo,all_struc(k),body,act,engine,ref);
        %     x(j)=all_act;
    end
end

mass_error_mat=zeros(N_act,N_condition);
for i=1:N_act
    for k=1:N_condition
        [all_act_out(k,i)]=actuator_sizing(all_act(k,i));
        if abs(all_act_out(k,i).mass-all_act(k,i).mass)>mass_error_mat(i,k)
            mass_error_mat(i,k)=abs(all_act_out(k,i).mass-all_act(k,i).mass);
        end
    end
end
mass_error=max(mass_error_mat(1,:));


% [act_out]=actuator_sizing(act);

% cost=all_engine(1).PSFC*(sum(all_trim_cost)+mass_error);
% s1=abs(all_state(1).AS/(all_engine(1).fuelFlow)+1)+abs(all_state(2).AS/(all_engine(2).fuelFlow+1))+abs(all_state(3).AS/(all_engine(3).fuelFlow)+1);
% s2=all_perf(4).ROC;
% s3=all_state(2).AS;
% 
% s4=sum(all_trim_cost)+mass_error;
% 
% cost=-s1-s2-s3+s4;

cost_trim=sum(abs(all_trim_cost));
cost_st=0;
cost_perf1=0;
cost_perf2=0;
cost_a=1;

for i=1:N_condition
    %     cost_st=cost_st+(all_perf(i).stability.SM+0.15).^2;
    if all_perf(i).stability.SM>-0.01
        cost_st=cost_st+abs(all_perf(i).stability.SM);
    end
     if all_perf(i).LOD<20
        cost_a=cost_a*1000;
     end
%      if all_perf(i).ROC<0
%         cost_a=cost_a*1000;
%     end
    
%     cost_perf1=cost_perf1+1/all_engine(i).fuelFlow;
end
% for i=1:1
%     cost_perf1=cost_perf1+all_engine(i).fuelFlow;
% end
% cost_perf2=1/(all_perf(2).ROC+1);
% if cost_perf2<=0
%     cost_perf2=-1000*cost_perf2;
% end
% cost_perf3=1/(all_state(4).AS+1);

%  cost=cost_trim*(cost_perf1+1)*(cost_perf2+1)*(cost_st+1)*cost_a;
%  cost=cost_trim*(cost_st+1)*cost_a;

cost_perf1=1/(abs(all_perf(1).Endurance)+1);
cost_perf2=1/(abs(all_perf(2).ROC)+1);


 cost=cost_trim^0.3*(cost_st+1)*cost_a*(cost_perf1*10+cost_perf2)*(mass_error+1);




% cost=cost_trim^2*(cost_st+1)*(cost_perf2)*(cost_perf3)*(cost_perf1+1)+mass_error;

% cost=sum(all_trim_cost);

% perf_cost1=abs(all_state(1).AS/all_engine(1).fuelFlow)+abs(all_state(2).AS/all_engine(2).fuelFlow)+abs(all_state(3).AS/all_engine(3).fuelFlow);
% perf_cost2=all_state(4).AS;
% perf_cost3=abs(all_perf(5).ROC);
% morph_cost=max(abs(morph_code(:,1)));
% 
% cost=1/(perf_cost1+perf_cost2+perf_cost3)*cost_trim*mass_error*(abs(morph_cost))^0.5;

% cost=cost_trim;


end

%
%

