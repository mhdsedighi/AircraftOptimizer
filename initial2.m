% clc
clear
close all
global doplot

state.g=9.81;
ref.b_ref=66*0.3048;
ref.S_ref=9.4;

W_to=10500*0.453592;
W_e=4900*0.453592;
W_pl=0*0.453592;
W_f=W_to-W_e-W_pl;
W_zf=W_to-W_f;

body.length=36.2*0.3048;
body.diam=4*0.3048;
body.inter= 1.5;         %Body interference factors
body.start=  [0 0 0];

engine.fanArea=pi*(9.2*0.3048/2)^2;
engine.Pshaftsls=575*745.7;
engine.assumptions.etaDisc                 = .92;
engine.assumptions.Q                       = 43e6; %J/kg Jet A fuel
engine.assumptions.efficiencyAtSeaLevel    = .846;
engine.assumptions.hMaxEfficiency          = 11277.6; % meters (37000 ft)
engine.assumptions.efficiencies            = {.4,@throttleefficiency,@altitudeefficiency};
engine.assumptions.powerlapse              = @powerlapse;

doplot=1;
input=[0 0];

%%% Geometry definition %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
geo.nwing=         3;           %Number of wings in the design,
%determines the number of ROWS in the
%geometry variables

geo.nelem=        [3 1 1];      %Number of partitions in the design,
%Determines the number of COLUMNS in
%the geometry variables

geo.ref_point=    [0 0 0];   %Position of the reference point in
%the aircraft coordinate system
%Moments are taken around this point

geo.CG=           [5 0 0]; 
% geo.CG=           [5.9061 0 0];    %Position of the center of gravity in
%the aircraft coordinate system.
%Rotations are made about this point

geo.c=            [5.4             %Root chord of each wing
    4.1  
    4.1]*0.3048;

geo.b=            [15 10 10         %Semispan of each partition of each wing
    10.8 0 0
    3.5 0 0]*0.3048;

geo.symetric=    [1 1 0];        %Symmetry bit for each wing
geo.startx=      [15.4 26.9 26.9]*0.3048;        %X coordinate of each wing apex
geo.starty=      [1.85 1.85 0]*0.3048;        %Y coordinate of each wing apex
geo.startz=      [0 0.5 -0.5]*0.3048;        %Z coordinate of each wing apex

geo.dihed=     [0 0 0            %Dihedral of each partition of each wing
    pi/6 0 0            %[rad]
    -pi/2 0 0];

geo.T=          [0.7 0.7 0.7           %Taper ratio of each partition.
    0.487 0 0
    0.6 0 0];

geo.SW=         [7 7 7          %c/4 sweep of each partition. [rad]
    10 0 0
    20 0 0]*pi/180;

geo.twist=[0 0 0 0
           0 0 0 0
           0 0 0 0]*pi/180;

geo.nx=        [3 3 3           %Number of panels in the X direction of
    3 0 0           %each partiton
    3 0 0];

geo.ny=        [5 5 5          %Number of panels in the Y direction of
    5 0 0           %each partiton
    5 0 0];

geo.flapped=    [0 0 0         %Partition flap bit. 1 for trailing edge flap
    1 0 0          % 1 for trailing edge flap
    0 0 0];        % 0 if no flap

geo.fnx=        [0 0 0          %Number of panels in the X direction of
    4 0 0          %the flap of each partiton
    0 0 0];

geo.fsym=      [1 0 0           %Control surface deflect symmetrically bit
    1 0 0
    0 0 0];

geo.fc=         [0.1 0.1 0.1          %Part of chord that is flapped of
    0.3 0 0           %each partition
    0 0 0];

geo.flap_vector= [0 0 0         %Flap deflection, [rad]
    -5 0 0         %right hand positive outboard
    0 0 0]*pi/180;


geo.airfoil=[{'0012'} {'0012'} {'0012'} {'0012'}     %Airfoil of section borders
    {'0012'} {'0012'} {''} {''}
    {'0012'} {'0012'} {''} {''}];


geo.meshtype=    [1 1 1                 %Type of mesh to be used
    1 1 1
    1 1 1];
%%%% End Aero structure definition %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

wing_index=1;
part_number=geo.nelem(wing_index);
cr=geo.c(wing_index);
S=0;
for p=1:part_number
    ct=cr*geo.T(wing_index,p);
    S=S+0.5*(cr+ct)*geo.b(wing_index,p);
    cr=ct;
end


%%% Define the state %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
state.AS=       194*0.44704;        %Airspeed m/s
state.alpha=       0.0611;    %Angle of attack, radians
state.betha=       0;         %Angle of sideslip, radians
state.P=       0;         %Rollrate, rad/s
state.Q=       0;         %pitchrate, rad/s
state.R=       0;         %yawrate, rad/s
state.adot=    0;         %Alpha time derivative rad/s
state.bdot=    0;         %Betha time derivative rad/s
state.ALT=       25000*0.3048;         %Altitude, m
state.pgcorr=       1;          %Apply prandtl glauert compressibility
state.phi=0;
state.theta=0;



[rho,a,~,mu]=ISAtmosphere(state.ALT);     %Calling International Standard atmosphere.
state.rho=rho;
state.Mach=state.AS/a;
state.q=0.5*(state.rho)*(state.AS)^2;
state.a=a;
state.mu=mu;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%%%%%%%
N=20;
geo.foil=cell(geo.nwing,geo.nwing,2);
for i=1:geo.nwing
    nsec=geo.nelem(i);
    for j=1:geo.nelem(i)
        geo.foil(i,j,1)=geo.airfoil(i,j);
        geo.TW(i,j,1)=geo.twist(i,j);
    end
    for j=1:geo.nelem(i)-1
        geo.foil(i,j,2)=geo.airfoil(i,j+1);
        geo.TW(i,j,2)=geo.twist(i,j+1);
    end
    geo.foil(i,nsec,2)=geo.airfoil(i,nsec+1);
end
for i=1:geo.nwing
    for j=1:geo.nelem(i)
        for k=1:2
            foil=geo.foil(i,j,k);
            [xz,~]=airfoil_read(foil,0,0,0,N);
            new_name=strjoin({'airfoil' num2str(i) num2str(j) num2str(k) '.DAT'});
            airfoil_write(xz,new_name)
            geo.foil(i,j,k)={new_name};
        end
    end
end


%%%%%%%%%%%%%%%%%%%%%%%


engine.number       =1;
engine.unitvector   =[-1 0 0];
% engine.pos         =[31  0  2.5]*0.3048;
engine.pos         =[31  0  0]*0.3048;
engine.throttle=0.3;
engine=engine_calc(engine,state);




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%state.PGcorr=0;
state.alpha=5*pi/180;



lattictype=1;%Standard VLM
%lattictype=0;%Tornado freestream following wake VLM

%%% Single aircraft selection.
[lattice,ref]=fLattice_setup2(geo,state,lattictype,ref);




%%%
[lattice.vertex_info] = get_vertex_info(geo,lattice);
%%%

%%%


wingno=1;
partno=3;
Raxle_local=[0 0 1];
Raxle_local=Raxle_local/norm(Raxle_local);
hinge_pos=[0 0 0];
alpha=deg2rad(15);
% isattached=1;
hinge_portion=0.5;
pivot_portion=0.5;

course_portion=0.5;

borderno=2;
% act(1).telescoping.cf=1;
% act(1).rotation.cf=1;
act(1).mass_guess=1;
act(1).pos=[0 0 0];
act(2).mass_guess=1;
act(2).pos=[0 0 0];
% act(2).telescoping.cf=1;
% act(2).rotation.cf=1;
% act(2).mass_guess=1;

act_num=1;
% geo1=geo;
% [geo,lattice,struc]=airfoil_bend(geo,lattice,struc,wingno,borderno,pivot_portion,alpha);
% geo2=geo;
% [geo,lattice,struc]=part_rotation(geo,lattice,struc,wingno,partno,Raxle,alpha,hinge_portion)
% [geo,lattice,struc,act]=telescoping(act,geo,lattice,struc,wingno,partno,course_portion)

%%%
% [lattice,ref]=fLattice_setup2(geo,state,lattictype);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Compute skin friction of wing system
[CD0_wing results.Re results.Swet results.Vol]=zeroliftdragpred(state.Mach,state.ALT,geo,ref);
%%% Compute blunt body drag increment
CD0_blunt=zldpblunt(state,body,ref);
%%% Summing up viscous drag
CD0=sum(sum(CD0_wing))+sum(CD0_blunt);
results.CD0=CD0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Rx=0.25;
Ry=0.38;
Rz=0.39;
Ixx=ref.b_ref^2*W_to*Rx^2/(4*state.g);
Iyy=body.length^2*W_to*Ry^2/(4*state.g);
Izz=((ref.b_ref+body.length)/2)^2*W_to*Rz^2/(4*state.g);
W_wing1=2.5*ref.S_ref*10.7639*0.453592; %%raymer method
n_ult=1.5;
b=ref.b_ref*3.28084;
S=ref.S_ref*3.28084*3.28084;
tc=0.12;
sweep=mean(geo.SW(1,:));
taper=mean(geo.T(1,:));
iw=1e-6*n_ult*b^3*sqrt(W_to*W_zf*2.20462*2.20462)/(tc*S*cos(sweep)^2)*(1+2*taper)/(1+taper);
W_wing2=S*(4.22+1.642*iw)*0.453592; %%shevell method
W_wing=W_wing1;
wing_vol=sum(results.Vol(1,:));

struc.geo=geo;
struc.XYZ=lattice.XYZ;
struc.vertex_info=lattice.vertex_info;
struc.pointmass_exist=0;
struc.rho(:,:)=ones(geo.nwing)*W_wing/wing_vol;
struc.pointmass.num=0;
struc.pointmass(1).m=1;
struc.pointmass(1).pos(:)=[0 1 0];
struc.m_body=0;
struc.cg_body=[0;0;0];
struc.inertia_body=zeros(3,3);
static_margin=0.25;
cg=[15.4*0.3048+0.75*geo.c(1,1);0;0];
ac=[15.4*0.3048+0.5*geo.c(1,1);0;0];
% geo.ref_point=ac';
struc.m_fuel=0;
struc.pos_fuel=[0 0 0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[struc]=massinertia(struc);
w_wing=struc.mass_all;
cg_wing=struc.cg_all;
inertia_wing=struc.inertia_all;
struc.m_body=W_e-w_wing;
struc.cg_body=(W_e*cg-w_wing*cg_wing)/struc.m_body;
struc.inertia_body(1,1)=Ixx-inertia_wing(1,1);
struc.inertia_body(2,2)=Iyy-inertia_wing(2,2);
struc.inertia_body(3,3)=Izz-inertia_wing(3,3);
struc.m_fuel=W_f;

[struc]=massinertia(struc);

geo.ref_point=ref.mac_pos;
geo.CG=struc.cg_all';
[lattice,ref]=fLattice_setup2(geo,state,lattictype,ref);

% 
% [geo,lattice,struc,act]=telescoping(1,geo,ref,lattice,results,state,struc,act,wingno,partno,course_portion)
% partno=2
% [geo,lattice,struc,act]=part_rotation(2,geo,lattice,results,struc,state,act,wingno,partno,Raxle_local,deg2rad(25),hinge_portion);



[results]=solver9(results,state,geo,lattice,ref);
[results]=coeff_create3(results,lattice,state,ref,geo);



lsize=size(results.F);
panels=lsize(1);

% geometryplot(lattice,geo,ref)

% cost= trim_cost(R,geo,state,results,struc,engine,ref)


derivatives;
stability;

if doplot
    plot_plane(body,geo,struc)
end

% plot_inertia_model(geo,lattice)


% cost1=abs(w_SP-8500);
% 
% Wto=93080;
% V=state.AS;
% CL=results.CL;
% CD=CD0;
% LOD=CL/CD
% Cj=0.68;
% Wbegin=Wto;
% Wend=Wbegin-0.2*Wbegin;
% % Endurance=1/Cj*(CL/CD)*log(Wbegin/Wend);
% % Range=V*CL/CD/Cj*log(Wbegin/Wend);
% 
% etta_prop=0.7;
% BSFC=2;
% 
% W=w_to-w_fuel;
% 
% Endurance=(CL/CD)*etta_prop/(BSFC*V)*log(Wbegin/Wend);
% Range=(CL/CD)*etta_prop/BSFC*log(Wbegin/Wend);
% ROC=sqrt(W/(0.5*rho*S*CL))*(T-D)/W;
% 
% cost2=-Range;
% 
% cost=cost1+cost2;
% 
% 
% 
