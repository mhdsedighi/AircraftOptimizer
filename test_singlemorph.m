
ang=20;
azi=90;
ele=0;

N_design=5;
N_morph=3;
N_trim=4;
N_act=1;
N_t=N_design+N_morph+N_trim+N_act;

close all
figure
hold on

struc.m_fuel=0;
struc.pos_fuel=[0 0 0];

clear all_state all_struc
for j=1:1
    all_state(j)=state;
    all_struc(j)=struc;
end

i_Xwing=[-0.5 0.5];
i_Zwing=[-0.2 0.2];
i_bfrac1=[0.01 0.5];
i_sweep_angle=[0 10]*pi/180;
i_twist_angle=[0 10]*pi/180;
% i_taper=[0.1 1]*pi/180;


% i_course_portion=[0 0.5];
i_rot_angle=[ang ang]*pi/180;
i_rot_azimuth=[azi azi]*pi/180;
i_rot_elevation=[ele ele]*pi/180;

i_alpha=[0 10]*pi/180;
i_elevator_angle=[-20 20]*pi/180;
i_throttle=[0 1.5];
V_service=357*0.44704;
i_AS=[0.6 1.2]*V_service/100;
i_actmass=[0 300]/100;


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


input=input1;
LB=lb1;
UB=ub1;
for i=1:1
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

design_code(1:N_design)=input(1:N_design);
actmass_code(1:N_act)=input(N_design+1:N_design+N_act);

inputmat=vector2matrix(input(N_design+N_act+1:end),1,N_trim+N_morph);

for k=1:1
    morph_code(k,1:N_morph)=inputmat(k,1:N_morph);
    trim_code(k,1:N_trim)=inputmat(k,N_morph+1:N_morph+N_trim);
end

[aero,all_perf,all_trim_cost,all_act,all_state,all_geo,all_struc,all_engine,all_ref] = compute_aircraft8(design_code,morph_code(k,:),trim_code(k,:),actmass_code,all_state(k),geo,all_struc(k),body,act,engine,ref);



plot_plane(body,all_geo(1),all_struc(1));
