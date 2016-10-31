close all
finilize=0;

N_condition=2;

plotmode.album=true;
plotmode.interlaced=true;

mode.base_design=false;
mode.morph_design=true;
mode.trim=true;

morphmode='only_sweep';
% morphmode='only_dihedral';
% morphmode='only_twist';

struc.m_fuel=0;
struc.pos_fuel=[0 0 0];

clear all_state all_struc
for j=1:N_condition
    all_state(j)=state;
    all_struc(j)=struc;
end

all_struc(1).m_fuel=W_f*0.8;
all_struc(2).m_fuel=W_f*0.8;


all_state(2).ALT=20000*0.3048;
opt_method='sa';

N_design=5;
N_morph=3;
N_trim=4;
N_act=1;
N_t=N_design+N_morph+N_trim+N_act;

i_Xwing=[-0.5 0.5];
i_Zwing=[-0.2 0.2];
i_bfrac1=[0.01 0.5];
i_sweep_angle=[-10 -10]*pi/180;
i_twist_angle=[0 0]*pi/180;
% i_taper=[0.1 1]*pi/180;


% i_course_portion=[0 0.5];
i_rot_angle=[-45 45]*pi/180;
i_rot_azimuth=[-20 20]*pi/180;
i_rot_elevation=[-20 20]*pi/180;

i_alpha=[0 10]*pi/180;
i_elevator_angle=[-20 20]*pi/180;
i_throttle=[0 1.5];
V_service=357*0.44704;
i_AS=[0.6 1.2]*V_service/100;

% i_actmass=[0 0;100 100];
i_actmass=[0 300]/100;

if mode.base_design
    N_condition=1;
end
if mode.morph_design
    if morphmode=='only_sweep'
        i_rot_azimuth=[0 0]*pi/180;
        i_rot_elevation=[90 90]*pi/180;
    elseif morphmode=='only_dihedral'
        i_rot_azimuth=[90 90]*pi/180;
        i_rot_elevation=[0 0]*pi/180;
    elseif morphmode=='only_twist'
        i_rot_azimuth=[0 0]*pi/180;
        i_rot_elevation=[0 0]*pi/180;
    end
else
    i_rot_angle=[0 0]*pi/180;
    i_rot_azimuth=[0 0]*pi/180;
    i_rot_elevation=[0 0]*pi/180;
    i_actmass=[0 0];
end

if ~mode.trim
    i_alpha=[5 5]*pi/180;
    i_elevator_angle=[0 0]*pi/180;
    i_throttle=[1 1];
    i_AS=[1 1]*V_service/100;
end


b1=[i_Xwing' i_Zwing' i_bfrac1' i_sweep_angle' i_twist_angle'];
b2=i_actmass';
b3=[i_rot_angle' i_rot_azimuth' i_rot_elevation'];
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

% b1=[i_Xwing' i_Zwing' i_sweep_angle' i_twist_angle'];
% b4=[i_alpha' i_elevator_angle' i_throttle' i_AS'];


% % %%%% for better guess
% % load('trim_results')
% % input1(1)=guess_design_code(1);
% % input1(2)=guess_design_code(2);
% % input1(4)=guess_design_code(3);
% % input1(5)=guess_design_code(4);
% % input2(N_morph+1)=guess_trim_code(1);
% % input2(N_morph+2)=guess_trim_code(2);
% % input2(N_morph+3)=guess_trim_code(3);
% % input2(N_morph+4)=guess_trim_code(4);
% % input2_prime(N_morph+1)=guess_trim_code(1);
% % input2_prime(N_morph+2)=guess_trim_code(2);
% % input2_prime(N_morph+3)=guess_trim_code(3);
% % input2_prime(N_morph+4)=guess_trim_code(4);
%%%% for better guess

input=input1;
LB=lb1;
UB=ub1;
for i=1:N_condition
    %     if i==1 || i==2 || i==3 %%% speed freezed
    %         LB=[LB lb2_prime];
    %         UB=[UB ub2_prime];
    %         input=[input input2_prime];
    %     else
    LB=[LB lb2];
    UB=[UB ub2];
    input=[input input2];
    
    %     end
    
end

fun=@(input)cost_func8(input,N_condition,N_design,N_morph,N_trim,N_act,all_state,geo,all_struc,body,act,engine,ref,finilize);

if ~finilize
    if opt_method=='sa'
        %%%%%% Starting with the default options for Simulated Annealing
        options = saoptimset;
        % %% Modifying options setting for Simulated Annealing
        % options = saoptimset(options,'TolFun', Optimization_Tolerance);
        % options = saoptimset(options,'MaxFunEvals', MaxFunctionEvaluation);
        % options = saoptimset(options,'MaxIter', MaxIteration);
        % options = saoptimset(options,'StallIterLimit', StallIterLimit_Data);
        % options = saoptimset(options,'InitialTemperature', InitialTemperature);
        % options = saoptimset(options,'ReannealInterval', ReannealInterval);
        options = saoptimset(options,'HybridInterval', 'end');
        options = saoptimset(options,'Display', 'iter');
        options = saoptimset(options,'PlotFcns', {  @saplotbestf @saplotbestx });
        % options = saoptimset(options,'PlotFcns',{@saplotbestx,@saplotbestf,@saplotx,@saplotf});
        options = saoptimset(options,'PlotInterval',3);
        options = saoptimset(options,'DisplayInterval',1);
        
        
        % %% Running Simulated Annealing for finding minimum of Electricity Cost
        [x,fval,exitflag,output] = simulannealbnd(fun,input,LB,UB,options);
    elseif opt_method=='ga'
        
        %
        %%% Starting with the default options
        options = gaoptimset;
        %%% Modifying options setting
        options = gaoptimset(options,'InitialPopulation', input);
        % options = gaoptimset(options,'StallGenLimit', Stall_Limit);
        % options = gaoptimset(options,'TolFun', Optimization_Tolerance);
        % options = gaoptimset(options,'TolCon',Costraint_Tolerance);
        options = gaoptimset(options,'PlotFcns', { @gaplotbestf @gaplotbestindiv });
        options = saoptimset(options,'PlotInterval',3);
        options = gaoptimset(options,'Display', 'iter');
        options = gaoptimset(options,'UseParallel', 'always');
        
        options = gaoptimset(options,'PopulationSize', 30);
        % options = gaoptimset(options,'EliteCount', EliteCount);
        % options = gaoptimset(options,'CrossoverFraction', CrossoverFraction);
        % options = gaoptimset(options,'MigrationInterval', MigrationInterval);
        % options = gaoptimset(options,'MigrationFraction', MigrationFraction);
        options = gaoptimset(options,'Generations', 1000);
        % options = gaoptimset(options,'PenaltyFactor', PenaltyFactor);
        % options = gaoptimset(options,'InitialPenalty', InitialPenalty);
        
        [x,fval,exitflag,output,population,score] = ga(fun,length(input),[],[],[],[],LB,UB,[],[],options);
    elseif opt_method=='ps'
        
        options = psoptimset;
        options.Display='Iter';
        options.PlotFcns={@psplotbestf @psplotbestx};
        x = patternsearch(fun,input,[],[],[],[],LB,UB,options);
        
    elseif opt_method=='psw'
        options = optimoptions('particleswarm','SwarmSize',5,'HybridFcn',@fmincon);
        options.Display='iter';
        options.UseParallel=true;
        options.PlotFcns={@pswplotbestf};
        rng default  % For reproducibility
        [x,fval,exitflag,output] = particleswarm(fun,length(input),LB,UB,options);
    end
else
    x=input;
end
% x=input;
finilize=1;
cost=cost_func8(x,N_condition,N_design,N_morph,N_trim,N_act,all_state,geo,all_struc,body,act,engine,ref,finilize);





%%%%test
% % % %
% design_code(1:N_design)=x(1:N_design);
% actmass_code(1:N_act)=x(N_design+1:N_design+N_act);
%
% inputmat=vector2matrix(x(N_design+N_act+1:end),N_condition,N_trim+N_morph);
%
% for j=1:N_condition
%     morph_code(j,1:N_morph)=inputmat(j,1:N_morph);
%     trim_code(j,1:N_trim)=inputmat(j,N_morph+1:N_morph+N_trim);
% end
%
% for j=1:N_condition
%     [ans_aero(j),ans_perf(j),ans_trim_cost(j),ans_act(j,:),ans_state2(j),ans_geo(j),ans_struc2(j),ans_engine(j),ans_ref(j)]=compute_aircraft8(design_code,morph_code(j,:),trim_code(j,:),actmass_code,ans_state(j),geo,ans_struc(j),body,act,engine,ref)
% end
% % %%%%%



if plotmode.album
    m=floor(sqrt(N_condition));
    n=floor(N_condition/m);
    if m*n<N_condition
        m=m+1;
    end
    figure
    count=0;
    for i=1:m
        for j=1:n
            count=count+1;
            if count<=N_condition;
                subplot(m,n,count)
                plot_plane(body,ans_geo(count),ans_struc(count));
            end
        end
    end
end
if plotmode.interlaced
    figure
    %    hold on
    for i=1:N_condition
        plot_plane(body,ans_geo(i),ans_struc(i));
    end
    
end

% max(abs(ans_act(1).movement))
% ans_act(1).energy
% max(abs(ans_act(1).M))



% close all
%
% plot_plane(body,ans_geo(1),ans_struc(1))
%
% figure
% plot_plane(body,ans_geo(2),ans_struc(2))
