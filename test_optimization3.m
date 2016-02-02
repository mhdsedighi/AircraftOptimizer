clc
clear all
close all

global doplot
results=[];
i=0;
j=0;
settings=config('startup');


quest=1;                %Simple state solution, could be something else
                        %but then you'll have to change below too,
                        %especially the "Load data" section.
JID='batchjob';     



doplot=0;
initial_guess=[0 0];
lower_bound=[-20 -20];
upper_bound=[20 20];

% doplot=1;
% sample_cost_func(initial_guess)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Optimization_Tolerance=1e-12;
MaxIteration=1000000;
MaxFunctionEvaluation=1000000;
StallIterLimit_Data=5000;
InitialTemperature=100;
ReannealInterval=100;
x0=initial_guess;

%%% Starting with the default options for Simulated Annealing
options = saoptimset;
%%% Modifying options setting for Simulated Annealing
options = saoptimset(options,'TolFun', Optimization_Tolerance);
options = saoptimset(options,'MaxFunEvals', MaxFunctionEvaluation);
options = saoptimset(options,'MaxIter', MaxIteration);
options = saoptimset(options,'StallIterLimit', StallIterLimit_Data);
options = saoptimset(options,'InitialTemperature', InitialTemperature);
options = saoptimset(options,'ReannealInterval', ReannealInterval);
options = saoptimset(options,'HybridInterval', 'end');
options = saoptimset(options,'Display', 'iter');
options = saoptimset(options,'PlotFcns', {  @saplotbestf @saplotbestx });
options = saoptimset(options,'PlotInterval',1);
options = saoptimset(options,'DisplayInterval',1);


% Running Simulated Annealing for finding minimum of Electricity Cost
[x,fval,exitflag,output] = simulannealbnd(@sample_cost_func5,x0,lower_bound,upper_bound,options);



doplot=1;
figure
sample_cost_func5(x)