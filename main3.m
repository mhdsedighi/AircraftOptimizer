
finilize=0;
N_condition=5;

struc.m_fuel=0;
struc.pos_fuel=[0 0 0];

clear all_state all_struc
for j=1:N_condition
    all_state(j)=state;
    all_struc(j)=struc;
end
 
all_struc(1).m_fuel=W_f*0.8;
all_struc(2).m_fuel=W_f*0.5;
all_struc(3).m_fuel=W_f*0.2;
all_struc(4).m_fuel=W_f*0.5;
all_struc(5).m_fuel=W_f*0.5;

all_state(5).ALT=20000*0.3048;


N_design=8;
N_morph=1;
N_trim=4;
N_act=1;
N_t=N_design+N_morph+N_trim+N_act;

i_Xwing=[-0.5 0.5];
i_Zwing=[-0.2 0.2];
i_bfrac1=[0.01 0.5];
i_sweep_angle=[-15 15]*pi/180;
i_twist_angle=[-15 15]*pi/180;
% i_taper=[0.1 1]*pi/180;


% i_course_portion=[0 0.5];
i_rot_angle=[-40 40]*pi/180;
i_Raxle_local=[0 0 0;1 1 1];

i_alpha=[0 10]*pi/180;
i_elevator_angle=[-20 20]*pi/180;
i_throttle=[0 1];
V_service=357*0.44704;
i_AS=[0.9 1.1]*V_service;

% i_actmass=[0 0;100 100];
i_actmass=[0 50]';



b1=[i_Xwing' i_Zwing' i_bfrac1' i_sweep_angle' i_twist_angle' i_Raxle_local];
b2=i_actmass;
b3=[i_rot_angle'];
b4=[i_alpha' i_elevator_angle' i_throttle' i_AS'];


bound1=[b1 b2];
bound2=[b3 b4];
bound2_prime=[b3 b4(:,1:3) V_service*ones(2,1)];
lb1=bound1(1,:);
ub1=bound1(2,:);
lb2=bound2(1,:);
ub2=bound2(2,:);
lb2_prime=bound2_prime(1,:);
ub2_prime=bound2_prime(2,:);

input1=(lb1+ub1)/2;
input2=(lb2+ub2)/2;
input2_prime=(lb2_prime+ub2_prime)/2;
%
% load('sample_input')
% input1=[ans_design_code ans_act_code];
% input2=[ans_morph_code(1,:) ans_trim_code(1,:)];
%
input=input1;
LB=lb1;
UB=ub1;
for i=1:N_condition
    if 0 % i==1 || i==2 || i==3
        LB=[LB lb2_prime];
        UB=[UB ub2_prime];
        input=[input input2_prime];
    else
        LB=[LB lb2];
        UB=[UB ub2];
        input=[input input2];
        
    end
    
    
end

fun=@(input)cost_func3(input,N_condition,N_design,N_morph,N_trim,N_act,all_state,geo,all_struc,body,act,engine,ref,finilize);


%%%%%
Optimization_Tolerance=1e-4;
Max_Generations=1000;
StallGenLimit=20;
PopulationSize=50;
% EliteCount=0.05*PopulationSize;
% EliteCount;
CrossoverFraction=0.8;
MigrationInterval=20;
MigrationFraction=0.2;
InitialPopulation=input;
InitialPenalty=10;
PenaltyFactor=100;
%%%%%

Cost_Dual=@(input)cost_func4(input,N_condition,N_design,N_morph,N_trim,N_act,all_state,geo,all_struc,body,act,engine,ref,finilize);


%%% Starting with the default options
% % options = gaoptimset;
% %%% Modifying options setting
% options = gaoptimset(options,'InitialPopulation', InitialPopulation);
% options = gaoptimset(options,'PopulationSize', PopulationSize);
% % options = gaoptimset(options,'EliteCount', EliteCount);
% options = gaoptimset(options,'CrossoverFraction', CrossoverFraction);
% options = gaoptimset(options,'MigrationInterval', MigrationInterval);
% options = gaoptimset(options,'MigrationFraction', MigrationFraction);
% options = gaoptimset(options,'Generations', Max_Generations);
% options = gaoptimset(options,'StallGenLimit', StallGenLimit);
% options = gaoptimset(options,'TolFun', Optimization_Tolerance);
% options = gaoptimset(options,'InitialPopulation', InitialPopulation);
% options = gaoptimset(options,'InitialPenalty', InitialPenalty);
% options = gaoptimset(options,'PenaltyFactor', PenaltyFactor);
% options = gaoptimset(options,'Display', 'iter');
% options = gaoptimset(options,'CrossoverFcn', {  @crossoverintermediate [] });
% options = gaoptimset(options,'Display', 'iter');
% % options = gaoptimset(options,'PlotFcns', {@gaplotpareto});
% options = gaoptimset(options,'PlotFcns', { @gaplotbestf @gaplotbestindiv });
% options = gaoptimset(options,'PlotInterval', 3);
% options = gaoptimset(options,'UseParallel', 'always');
% Nvars=length(input);

% [x,fval,exitflag,output,population,score] = ga(fun,length(input),[],[],[],[],LB,UB,[],[],options);

% [x,fval,exitflag,output,population,score] = gamultiobj(Cost_Dual,Nvars,[],[],[],[],LB,UB,[],options);

%%%%%% Starting with the default options for Simulated Annealing
% options = saoptimset;
% % %% Modifying options setting for Simulated Annealing
% % options = saoptimset(options,'TolFun', Optimization_Tolerance);
% % options = saoptimset(options,'MaxFunEvals', MaxFunctionEvaluation);
% % options = saoptimset(options,'MaxIter', MaxIteration);
% % options = saoptimset(options,'StallIterLimit', StallIterLimit_Data);
% % options = saoptimset(options,'InitialTemperature', InitialTemperature);
% % options = saoptimset(options,'ReannealInterval', ReannealInterval);
% options = saoptimset(options,'HybridInterval', 'end');
% options = saoptimset(options,'Display', 'iter');
% options = saoptimset(options,'PlotFcns', {  @saplotbestf @saplotbestx });
% % options = saoptimset(options,'PlotFcns',{@saplotbestx,@saplotbestf,@saplotx,@saplotf});
% options = saoptimset(options,'PlotInterval',3);
% options = saoptimset(options,'DisplayInterval',1);
% 
% 
% % %% Running Simulated Annealing for finding minimum of Electricity Cost
% [x,fval,exitflag,output] = simulannealbnd(fun,input,LB,UB,options);



%%%  (global search)
% options = optimoptions(@fmincon,'Algorithm','interior-point','MaxFunEvals',5000,'TolFun',1e-5,'Display','iter');
% problem = createOptimProblem('fmincon','objective',fun,'x0',input,'lb',LB,'ub',UB,'options',options);
% gs = GlobalSearch;
% % gs.Display='iter';
% [optimum_point,min_cost]=run(gs,problem);



% x=input;
% finilize=1;
% cost=cost_func3(x,N_condition,N_design,N_morph,N_trim,N_act,all_state,geo,all_struc,body,act,engine,ref,finilize);



% options = optimoptions('particleswarm','SwarmSize',5,'HybridFcn',@fmincon);
% options.Display='iter';
% options.UseParallel=true;
% options.PlotFcns={@pswplotbestf};
% rng default  % For reproducibility
% [x,fval,exitflag,output] = particleswarm(fun,length(input),LB,UB,options)

%%%%test
% % % % 
% % design_code(1:N_design)=input(1:N_design);
% % actmass_code(1:N_act)=input(N_design+1:N_design+N_act);
% % 
% % inputmat=vector2matrix(input(N_design+N_act+1:end),N_condition,N_trim+N_morph);
% % 
% % for j=1:N_condition
% %     morph_code(j,1:N_morph)=inputmat(j,1:N_morph);
% %     trim_code(j,1:N_trim)=inputmat(j,N_morph+1:N_morph+N_trim);
% % end
% % 
% % for j=1:N_condition
% % [all_aero(j),all_perf(j),all_trim_cost(j),all_act(j,:),all_state2(j),all_geo(j),all_struc2(j),all_engine(j),all_ref(j)]=compute_aircraft(design_code,morph_code(j,:),trim_code(j,:),actmass_code,all_state(j),geo,all_struc(j),body,act,engine,ref)
% % end
% %%%%%






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
