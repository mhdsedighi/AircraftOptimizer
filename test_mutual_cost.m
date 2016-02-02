

N_char=2;
Nc=2;
Nd=2;


current_performance=zeros(Nc,N_char);
desired_performance=zeros(Nd,N_char);
importance=ones(1,N_char);

current_performance(1,1)=1.2;
current_performance(1,2)=2.2;

current_performance(2,1)=1.3;
current_performance(2,2)=2.3;



desired_performance(1,1)=1.3;
desired_performance(1,2)=2.3;

desired_performance(2,1)=1.5;
desired_performance(2,2)=2.4;



[ cost ] = mutual_cost(current_performance,desired_performance,importance,Nc,Nd)

