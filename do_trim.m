
close all

%%%design
interval_alpha=[0 10]*pi/180;
interval_elevator_angle=[-10 10]*pi/180;
interval_wing_angle=[-10 10]*pi/180;
interval_throttle=[0 1];
interval_Xwing=[-0.5 0.5];
interval_Zwing=[-0.2 0.2];
interval_rot1=[0 20]*pi/180;

design_lb=[interval_alpha(1) interval_elevator_angle(1) interval_wing_angle(1) interval_throttle(1) interval_Xwing(1) interval_Zwing(1) interval_rot1(1)];
design_ub=[interval_alpha(2) interval_elevator_angle(2) interval_wing_angle(2) interval_throttle(2) interval_Xwing(2) interval_Zwing(2) interval_rot1(2)];

design_initial_guess=[0 0 0 0.5 0 0 0];



% [ cost ] = trim_cost(initial_guess,geo,state,results,struc,engine,ref)
finilize=0;
fun=@(input)trim_cost(input,finilize,geo,state,results,struc,act,engine,ref);
% [x,fval,exitflag] = fminbnd(fun,lb,ub)
% [ cost ] = trim_cost(input,geo,state,results,struc,engine,ref)

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
[x,fval,exitflag,output] = simulannealbnd(fun,initial_guess,lb,ub,options);

finilize=1;
cost=trim_cost(x,finilize,geo,state,results,struc,act,engine,ref);

close all
% geometryplot(lattice,geo,ref)
% figure
plot_plane(body,struc)

% options=psoptimset;
% 
% options = psoptimset(options,'PlotFcns', {  @psplotbestf @psplotbestx });
% options = psoptimset(options,'Display', 'iter');
% x = patternsearch(fun,initial_guess,[],[],[],[],lb,ub,[],options)