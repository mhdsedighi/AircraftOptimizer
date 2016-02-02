
finilize=0;
N_condition=1;

struc.m_fuel=0.9*W_f;
struc.pos_fuel=[0 0 0];

% state=change_state(h,v);
for j=1:N_condition
    all_state(j)=state;
    all_struc(j)=struc;
end
 
% all_state(1).ALT= 20000*0.3048; 


N_design=4;
N_morph=0;
N_trim=4;
N_act=0;
N_t=N_design+N_morph+N_trim+N_act;

i_Xwing=[-0.5 0.5];
i_Zwing=[-0.2 0.2];
i_sweep_angle=[-10 10]*pi/180;
i_twist_angle=[-10 10]*pi/180;


% i_course_portion=[0 0.5];
% i_rot_angle=[-30 30]*pi/180;
% i_Raxle_local=[0 0 0;1 1 1];

i_alpha=[0 10]*pi/180;
i_elevator_angle=[-10 10]*pi/180;
i_throttle=[0 1];
i_AS=[0.5 0.6]*357*0.44704;

% i_actmass=[0 0;100 100];
% i_actmass=[0 100]';


act(1).mass_guess=0;
act(1).pos=[0 0 0];
act(2).mass_guess=0;
act(2).pos=[0 0 0];


b1=[i_Xwing' i_Zwing' i_sweep_angle' i_twist_angle'];
b4=[i_alpha' i_elevator_angle' i_throttle' i_AS'];


bound1=[b1];
bound2=[b4];
lb1=bound1(1,:);
ub1=bound1(2,:);
lb2=bound2(1,:);
ub2=bound2(2,:);

input1=(lb1+ub1)/2;
input2=(lb2+ub2)/2;

input=input1;
LB=lb1;
UB=ub1;
for i=1:N_condition
    LB=[LB lb2];
    UB=[UB ub2];
    input=[input input2];
end

fun=@(input)cost_func2(input,N_condition,N_design,N_morph,N_trim,N_act,all_state,geo,all_struc,body,act,engine,ref,finilize);

%%% Starting with the default options for Simulated Annealing
options = saoptimset;
%%% Modifying options setting for Simulated Annealing
% options = saoptimset(options,'TolFun', Optimization_Tolerance);
% options = saoptimset(options,'MaxFunEvals', MaxFunctionEvaluation);
% options = saoptimset(options,'MaxIter', MaxIteration);
% options = saoptimset(options,'StallIterLimit', StallIterLimit_Data);
% options = saoptimset(options,'InitialTemperature', InitialTemperature);
% options = saoptimset(options,'ReannealInterval', ReannealInterval);
% options = saoptimset(options,'HybridInterval', 'end');
options = saoptimset(options,'Display', 'iter');
options = saoptimset(options,'PlotFcns', {  @saplotbestf @saplotbestx });
% % options = saoptimset(options,'PlotFcns',{@saplotbestx,@saplotbestf,@saplotx,@saplotf});
% options = saoptimset(options,'PlotInterval',100);
% options = saoptimset(options,'DisplayInterval',100);


%%% Running Simulated Annealing for finding minimum of Electricity Cost
[x,fval,exitflag,output] = simulannealbnd(fun,input,LB,UB,options);


% options = psoptimset;
% options.Display='Iter';
% x = patternsearch(fun,input,[],[],[],[],LB,UB,options)



% %%% Starting with the default options
% options = gaoptimset;
% %%% Modifying options setting
% options = gaoptimset(options,'InitialPopulation', input);
% % options = gaoptimset(options,'StallGenLimit', Stall_Limit);
% % options = gaoptimset(options,'TolFun', Optimization_Tolerance);
% % options = gaoptimset(options,'TolCon',Costraint_Tolerance);
% options = gaoptimset(options,'PlotFcns', { @gaplotbestf @gaplotbestindiv });
% options = saoptimset(options,'PlotInterval',3);
% options = gaoptimset(options,'Display', 'iter');
% options = gaoptimset(options,'UseParallel', 'always');
% 
% options = gaoptimset(options,'PopulationSize', 40);
% % options = gaoptimset(options,'EliteCount', EliteCount);
% % options = gaoptimset(options,'CrossoverFraction', CrossoverFraction);
% % options = gaoptimset(options,'MigrationInterval', MigrationInterval);
% % options = gaoptimset(options,'MigrationFraction', MigrationFraction);
% % options = gaoptimset(options,'Generations', Max_Generations);
% % options = gaoptimset(options,'PenaltyFactor', PenaltyFactor);
% % options = gaoptimset(options,'InitialPenalty', InitialPenalty);

% [x,fval,exitflag,output,population,score] = ga(fun,length(input),[],[],[],[],LB,UB,[],[],options);

% % x=input;
finilize=1;
cost=cost_func2(x,N_condition,N_design,N_morph,N_trim,N_act,all_state,geo,all_struc,body,act,engine,ref,finilize);


%%%%test

% design_code(1:N_design)=input(1:N_design);
% actmass_code(1:N_act)=input(N_design+1:N_design+N_act);
% 
% inputmat=vector2matrix(input(N_design+N_act+1:end),N_condition,N_trim+N_morph);
% 
% for j=1:N_condition
%     morph_code(j,1:N_morph)=inputmat(j,1:N_morph);
%     trim_code(j,1:N_trim)=inputmat(j,N_morph+1:N_morph+N_trim);
% end
% j=1;
% 
% compute_aircraft2(design_code,trim_code(j,:),all_state(j),geo,all_struc(j),body,act,engine,ref);
%%%%%





% 
% max(abs(ans_act(1).movement))
% ans_act(1).energy
% max(abs(ans_act(1).M))
% 
% close all
% 
% plot_plane(body,ans_geo(1),ans_struc(1))
% 
% figure
% plot_plane(body,ans_geo(2),ans_struc(2))


na=ans_state(1).q*ans_aero(1).CL_a/(struc.mass_all/ref.S_ref);
