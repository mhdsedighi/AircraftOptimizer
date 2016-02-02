clc
clear all
close all

global doplot CL CD
results=[];
i=0;
j=0;
settings=config('startup');


quest=1;                %Simple state solution, could be something else
                        %but then you'll have to change below too,
                        %especially the "Load data" section.
JID='batchjob';     



doplot=0;
initial_guess=[5 0.5 0.5 0.5];
lower_bound=[-20 0.01 0.01 0.01];
upper_bound=[20 1 1 1];

% doplot=1;
% sample_cost_func(initial_guess)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Optimization_Tolerance=1e-12;
% MaxIteration=1000000;
% MaxFunctionEvaluation=1000000;
% StallIterLimit_Data=5000;
% InitialTemperature=100;
% ReannealInterval=100;
x0=initial_guess;

% initial_guess=mean([x1;x3]);
% max_mwt_value=3000;
% lower_bound=zeros(1,Nvars);
% upper_bound=max_mwt_value*ones(1,Nvars);
%%%%%

%%%%%
Optimization_Tolerance=1e-4;
Max_Generations=1000;
StallGenLimit=20;
PopulationSize=100;
EliteCount=0.05*PopulationSize;
CrossoverFraction=0.8;
MigrationInterval=20;
MigrationFraction=0.2;
InitialPopulation=initial_guess;
InitialPenalty=10;
PenaltyFactor=100;
%%%%%

Cost_Dual=@(input)sample_cost_func6(input);


%%% Starting with the default options
options = gaoptimset;
%%% Modifying options setting
options = gaoptimset(options,'InitialPopulation', InitialPopulation);
options = gaoptimset(options,'PopulationSize', PopulationSize);
options = gaoptimset(options,'EliteCount', EliteCount);
options = gaoptimset(options,'CrossoverFraction', CrossoverFraction);
options = gaoptimset(options,'MigrationInterval', MigrationInterval);
options = gaoptimset(options,'MigrationFraction', MigrationFraction);
options = gaoptimset(options,'Generations', Max_Generations);
options = gaoptimset(options,'StallGenLimit', StallGenLimit);
options = gaoptimset(options,'TolFun', Optimization_Tolerance);
options = gaoptimset(options,'InitialPopulation', InitialPopulation);
options = gaoptimset(options,'InitialPenalty', InitialPenalty);
options = gaoptimset(options,'PenaltyFactor', PenaltyFactor);
options = gaoptimset(options,'Display', 'iter');
options = gaoptimset(options,'CrossoverFcn', {  @crossoverintermediate [] });
options = gaoptimset(options,'Display', 'iter');
options = gaoptimset(options,'PlotFcns', {@gaplotpareto});
options = gaoptimset(options,'PlotInterval', 1);
Nvars=length(initial_guess);

[x,fval,exitflag,output,population,score] = gamultiobj(Cost_Dual,Nvars,[],[],[],[],lower_bound,upper_bound,[],options);



[m,n]=size(x);


m1=floor(sqrt(m));
m2=m-m1;

doplot=1;
figure
for i=1:m
subplot(m1,m2,i)
sample_cost_func6(x(i,:));
end


figure
hold on
for i=1:m
sample_cost_func6(x(i,:));
end