% function [results]=tornadobatch(ac_name,state_name);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% tornadobatch, subsidary function to TORNADO	
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%	ac_name = name of geometry file (string)
%	state_name=name of state file (string)
%	JID=job identifier, output filename (string)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc
clear all
close all

results=[];
i=0;
j=0;
settings=config('startup')

% cd(settings.acdir)
% 	load('L_45');
% load('X31');
% cd(settings.hdir)

% cd(settings.sdir)
% 	load('teststate');
% cd(settings.hdir)


quest=1;                %Simple state solution, could be something else
                        %but then you'll have to change below too,
                        %especially the "Load data" section.
JID='batchjob';             

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CHANGE GEOMETRY
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Here you may enter whatever changes to the geometry you like
%
% Examples:
%   Change number of panels for easy grid convergence.
%   Change a rudder setting, or tailplane twist for trim computations
%   Change the geometry in any other way.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % geo=geo;
% % geo.nx(1,1)=1;
% % geo.nx(1,2)=2;
% % geo.nx(1,3)=4;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%% Geometry definition %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
geo.nwing=         3;           %Number of wings in the design, 
                                %determines the number of ROWS in the
                                %geometry variables 
                                
geo.nelem=        [3 1 1];      %Number of partitions in the design,           
                                %Determines the number of COLUMNS in
                                %the geometry variables  

geo.ref_point=    [0.33 0 0];   %Position of the reference point in 
                                %the aircraft coordinate system
                                %Moments are taken around this point
                
geo.CG=           [0.25 0 0];    %Position of the center of gravity in 
                                 %the aircraft coordinate system.
                                 %Rotations are made about this point

geo.c=            [0.7             %Root chord of each wing 
                   0.4 
                   0.4];
               
geo.b=            [2 2 1         %Semispan of each partition of each wing   
                   1 0 0
                   0.5 0 0];

geo.symetric=    [1 1 0];        %Symmetry bit for each wing
geo.startx=      [1 4 4];        %X coordinate of each wing apex   
geo.starty=      [0 0 0];        %Y coordinate of each wing apex
geo.startz=      [0 0 0];        %Z coordinate of each wing apex
               
geo.dihed=     [0 0 0            %Dihedral of each partition of each wing    
                pi/5 0 0            %[rad]
                -pi/2 0 0];
             
geo.T=          [0.7 0.7 0.7           %Taper ratio of each partition.
                0.9 0 0
                0.9 0 0];

geo.SW=         [0 0 0          %c/4 sweep of each partition. [rad]
                0 0 0
                0 0 0];
            
geo.TW(:,:,1)=  [0 pi/12 0          %Twist of partition inboard section
                0 0 0
                0 0 0];
            
geo.TW(:,:,2)=  [pi/12 0 0          %Twist of partition outboard section
                0 0 0
                0 0 0];
            
geo.nx=        [4 4 4           %Number of panels in the X direction of 
                4 0 0           %each partiton
                4 0 0];
    
 geo.ny=        [2 2 2          %Number of panels in the Y direction of 
                2 0 0           %each partiton
                2 0 0];
            
 geo.flapped=    [1 1 1         %Partition flap bit. 1 for trailing edge flap
                 0 0 0          % 1 for trailing edge flap
                 0 0 0];        % 0 if no flap
  
geo.fnx=        [2 2 2          %Number of panels in the X direction of
                 0 0 0          %the flap of each partiton
                 0 0 0];
            
geo.fsym=      [1 0 0           %Control surface deflect symmetrically bit
                0 0 0
                0 0 0];
          
geo.fc=         [0.1 0.1 0.3          %Part of chord that is flapped of
                0 0 0           %each partition    
                0 0 0];
          
geo.flap_vector= [0 0 20         %Flap deflection, [rad]    
                  0 0 0         %right hand positive outboard
                  0 0 0];

geo.foil(:,:,1)= [{'0012'} {'0012'} {'0012'}     %Airfoil file inboard profile
                  {'0012'} {'0012'} {'0012'}
                  {'0012'} {'0012'} {'0012'}];
              
geo.foil(:,:,2)= [{'0012'} {'0012'} {'0012'}      %Airfoil file outboard profile
                  {'0012'} {'0012'} {'0012'}
                  {'0012'} {'0012'} {'0012'}];
              
geo.meshtype=    [1 1 1                 %Type of mesh to be used
                  1 1 1
                  1 1 1];
%%%% End Aero structure definition %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%              

%%% Define the state %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
state.AS=       10;        %Airspeed m/s
state.alpha=       0.0611;    %Angle of attack, radians
state.betha=       0;         %Angle of sideslip, radians
state.P=       0;         %Rollrate, rad/s
state.Q=       0;         %pitchrate, rad/s
state.R=       0;         %yawrate, rad/s
state.adot=    0;         %Alpha time derivative rad/s
state.bdot=    0;         %Betha time derivative rad/s
state.ALT=       0;         %Altitude, m
%  state.rho=       1.225;     %Desity, kg/m^3
state.pgcorr=       1;          %Apply prandtl glauert compressibility 
                                   %correction

[rho,a,~,~]=ISAtmosphere(state.ALT);     %Calling International Standard atmosphere.
state.rho=rho;
state.Mach=state.AS/a;
state.g=9.81;
state.q=0.5*(state.rho)*(state.AS)^2;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
struc.rho(:,:)=1000*ones(geo.nwing);


%%%%%%%%%%%%%%%%%%%%%%%

body.length= [5];             %Lengths of blunt bodies such as:  [Body  Nacelle Nacelle]
body.diam=   [0.4];     %Diameter of blunt bodies in the same order as the lengths
body.inter=  [1];         %Body interference factors    
body.start=  [0 0 0];

engine.number       =1;
engine.moments      =[ 0  1000  3000];                    
engine.xyz          =[ 5  0  0];
engine.thrust       =[1000];    
engine.vector       =[-1 0 0];
engine.unitvector   =[-1 0 0];










%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% CHANGE STATE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Here you may enter whatever changes to the state you like
%
% Examples:
%   Change angle of attack or sideslip.
%   Change altitude, airspeed or ainything else
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%state.PGcorr=0;
state.alpha=4*pi/180;
geo.flap_vector=deg2rad(geo.flap_vector);



lattictype=1;%Standard VLM
%lattictype=0;%Tornado freestream following wake VLM

%%% Single aircraft selection.
[lattice,ref]=fLattice_setup2(geo,state,lattictype);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Compute skin friction of wing system
[CD0_wing results.Re results.Swet results.Vol]=zeroliftdragpred(state.Mach,state.ALT,geo,ref);
%%% Compute blunt body drag increment
CD0_blunt=zldpblunt(state.Mach,state.ALT,body,ref);
%%% Summing up viscous drag
CD0=sum(sum(CD0_wing))+sum(CD0_blunt)
results.CD0=CD0
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


[results]=solver9(results,state,geo,lattice,ref);
[results]=coeff_create3(results,lattice,state,ref,geo)

lsize=size(results.F);
panels=lsize(1);

% geometryplot(lattice,geo,ref)
[struc]=massinertia(geo,lattice,struc)
plot_plane(geo,body,lattice)

derivatives
stability


% plot_inertia_model(geo,lattice)
% showroom
% % 
% % DP=[-70 0 70 140
% %    70 140 210 280
% %     0 0 0 0];   %Delta position vector.
% % 
% % [void nAircraft]=size(DP);
% % 
% % 
% % [lattice]=fmultLattice(lattice,DP);
% % 
% % 
% % 
% % ref=ref;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % [results]=solver9(results,state,geo,lattice,ref);
% % [RT]=coeff_create3(results,lattice,state,ref,geo);
% % % 
% % % 
% % % [a b c]=size(results.F);
% % % 
% % % 
% % % for i=1:nAircraft+1;
% % % 
% % %     index=[((i-1)*panels+1):i*panels];
% % %     
% % %     
% % % R1.dwcond=results.dwcond;
% % % R1.F=results.F(index,:,:);
% % % 
% % % R1.FORCE=sum(results.F(index,:,:),1);
% % % R1.M=results.M(index,:,:)
% % % R1.MOMENTS=sum(results.M(index,:,:),1);
% % % R1.gamma=results.gamma(index,:,:);
% % % 
% % % 
% % % R(i)=coeff_create3(R1,lattice,state,ref,geo);
% % % 
% % % end
% % % 
% % % format compact
% % % Basedrag=R0.CD
% % % 
% % % for i=1:nAircraft+1
% % %     Drags(i)=R(i).CD;
% % % end
% % % Drags
% % % gain=1-Drags/Basedrag
% % % 
% % % % geometryplot(lattice,geo,ref);
% % % 
% % % 
% % % 
% % % 
% % % 
% % % 





    
    
    
    