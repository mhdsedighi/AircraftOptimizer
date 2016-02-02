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


%% Geometry definition %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
geo.nwing=         2;           %Number of wings in the design, 
                                %determines the number of ROWS in the
                                %geometry variables 
                                
geo.nelem=        [1 1];      %Number of partitions in the design,           
                                %Determines the number of COLUMNS in
                                %the geometry variables  

geo.ref_point=    [0 0 0];   %Position of the reference point in 
                                %the aircraft coordinate system
                                %Moments are taken around this point
                
geo.CG=           [0.25 0 0];    %Position of the center of gravity in 
                                 %the aircraft coordinate system.
                                 %Rotations are made about this point

geo.c=            [1             %Root chord of each wing 
                   1];
               
geo.b=            [1         %Semispan of each partition of each wing   
                   1];

geo.symetric=    [1 1];        %Symmetry bit for each wing
geo.startx=      [0 10];        %X coordinate of each wing apex   
geo.starty=      [0 1];        %Y coordinate of each wing apex
geo.startz=      [0 0.5];        %Z coordinate of each wing apex
               
geo.dihed=     [pi/10            %Dihedral of each partition of each wing    
                pi/10];
             
geo.T=          [1           %Taper ratio of each partition.
                 1];

geo.SW=         [pi/6          %c/4 sweep of each partition. [rad]
                pi/6];
            
geo.TW(:,:,1)=  [0          %Twist of partition inboard section
                pi/6];
            
geo.TW(:,:,2)=  [pi/6          %Twist of partition outboard section
                0];
            
geo.nx=        [2
                2];%Number of panels in the X direction of ];
    
 geo.ny=        [2          %Number of panels in the Y direction of          %each partiton
                 2];
            
 geo.flapped=    [0         %Partition flap bit. 1 for trailing edge flap
                  0];        % 0 if no flap
  
geo.fnx=        [0          %Number of panels in the X direction of
                 0];
            
geo.fsym=      [1           %Control surface deflect symmetrically bit
                1];
          
geo.fc=         [0.1          %Part of chord that is flapped of  
                0];
          
geo.flap_vector= [0         %Flap deflection, [rad]    
                  0];

geo.foil(:,:,1)= [{'0010'}     %Airfoil file inboard profile
                  {'0010'}];
              
geo.foil(:,:,2)= [{'0010'}     %Airfoil file inboard profile
                  {'0010'}];
              
geo.meshtype=    [1                 %Type of mesh to be used
                  1];
%%%% End Aero structure definition %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%              

%%% Define the state %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        state.AS=       1;        %Airspeed m/s
     state.alpha=       0.0611;    %Angle of attack, radians
     state.betha=       0;         %Angle of sideslip, radians
         state.P=       0;         %Rollrate, rad/s
         state.Q=       0;         %pitchrate, rad/s
         state.R=       0;         %yawrate, rad/s
         state.adot=    0;         %Alpha time derivative rad/s
         state.bdot=    0;         %Betha time derivative rad/s
       state.ALT=       0;         %Altitude, m
       state.rho=       1.225;     %Desity, kg/m^3
    state.pgcorr=       1;          %Apply prandtl glauert compressibility 
                                   %correction

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% CHANGE STATE
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


lattictype=1;%Standard VLM
%lattictype=0;%Tornado freestream following wake VLM
%geo.ny(1,1:2)=2;



%%% Single aircraft selection.
[lattice1,ref]=fLattice_setup2(geo,state,lattictype);


% lattice1.VORTEX(:,:,:)=lattice1.VORTEX(1,1,1);


wingno=1;
Raxle=[0 0 1];
hinge_pos=[0 0 0];
alpha=pi/4;

% [lattice1]=wingrotation_m(wingno,geo,lattice1,Raxle,hinge_pos,alpha);

[results]=solver9(results,state,geo,lattice1,ref);
[R0]=coeff_create3(results,lattice1,state,ref,geo);

lsize=size(results.F);
panels=lsize(1);

geometryplot(lattice1,geo,ref)




% % 
% % DP=[-70 0 70 140
% %    70 140 210 280
% %     0 0 0 0];   %Delta position vector.
% % 
% % [void nAircraft]=size(DP);
% % 
% % 
% % [lattice]=fmultLattice(lattice1,DP);
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
% % % R(i)=coeff_create3(R1,lattice1,state,ref,geo);
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


% % % figure
% % % hold on
% % % % plot3(lattice1.VORTEX(:,:,1),lattice1.VORTEX(:,:,2),lattice1.VORTEX(:,:,2),'k.')
% % % % plot3(lattice1.XYZ(:,:,1),lattice1.XYZ(:,:,2),lattice1.XYZ(:,:,3),'.')
% % % 
% % % plot3(lattice1.COLLOC(:,1),lattice1.COLLOC(:,2),lattice1.COLLOC(:,3),'.')

    
    
    
    