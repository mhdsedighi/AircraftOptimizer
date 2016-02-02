function engine=engine_calc(engine,state)

% 
% fanArea=engine.fanArea;
% Pshaftsls=engine.Pshaftsls;
% 
% throttle=engine.throttle;
% % M=state.Mach;
% % PshaftAvail = lapse.*Pshaftsls;
h=state.ALT; %m
if h>15243
    h=15243;
end
% % rho=state.rho;
V=state.AS;
% 
[~,a] = atmos(h);
M=V/a;

T0=42000;
% TSFC0=0.605*0.224809*0.453592;
TSFC0=0.061692796;
[T,TSFC]=jetengine(h,M,T0,TSFC0);


engine.Thrust=T*engine.throttle;
engine.TSFC=TSFC*6.5;
engine.fuelFlow=TSFC*T;   %%1/h

% 
% % % engine.assumptions.etaDisc                 = .92;
% % % engine.assumptions.Q                       = 43e6; %J/kg Jet A fuel
% % % % engine.assumptions.Q                       = 120e6; %MJ/kg liquid hydrogen
% % % engine.assumptions.efficiencyAtSeaLevel    = .846;
% % % engine.assumptions.hMaxEfficiency          = 11277.6; % meters (37000 ft)
% % % engine.assumptions.efficiencies            = {.4,@throttleefficiency,@altitudeefficiency};
% % % engine.assumptions.powerlapse              = @powerlapse;
% 
% lapse = engine.assumptions.powerlapse(h,M,engine.assumptions);
% PshaftAvail = lapse.*Pshaftsls;
% % [Tavail,etaProp] = actuatordisc('computeT',PshaftAvail,rho,fanArea,V,engine.assumptions.etaDisc);
% % [PSFC,eCoreTotal,eCore] = calculatepsfc(h,M,throttle,engine.assumptions);
% [Tavail,etaProp] = actuatordisc('computeT',PshaftAvail,rho,fanArea,V,engine.assumptions.etaDisc);
% [PSFC,eCoreTotal,eCore] = calculatepsfc(h,M,throttle,engine.assumptions);
% PshaftReq=throttle*PshaftAvail;
% fuelFlow = PshaftReq.*PSFC;
% 
% 
% engine.Tavail=Tavail;
% engine.PSFC=PSFC;
% engine.PshaftReq=PshaftReq;
% engine.fuelFlow=fuelFlow;
% engine.etaProp=etaProp;
% engine.Pout=etaProp*PshaftReq;
% engine.Thrust=engine.Pout/V;
% engine.TSFC=fuelFlow./engine.Thrust;

end


function [T,TSFC]=jetengine(alt,mach,T0,TSFC0)
%program for thrust and TSFC of a low-bypass, afterburning turbofan at
%maximum power setting
M=[0 0.25 0.5 0.75 1 1.25 1.5 1.75 2 2.25]; %Mach number
h=[0 10 20 30 36 40 50]*1000/3.28; %std. altitude (m)
Thrust=[30 21.5 15 10 8 6.5 4;
29 21 14.5 9.8 7.5 6 3.8;
32 22.5 16 10.5 8.5 7 4.5;
33 28 19 12.5 10 8 5;
35 29 23.5 16 12.5 10 6;
37 31 25.5 21 16 13 8.5;
42.5 35 28 22.5 19.5 15.5 9.2;
43.5 38 33 25 21.5 17.5 10.5;
46 39 34 28 24.5 19 11.5;
48 42 35 29 26 21.5 13]*1000*9.8/2.2; %thrust (N)
TSFC=[1.64 1.66 1.68 1.7 1.71 1.71 1.71;
1.74 1.76 1.77 1.78 1.79 1.79 1.79;
1.78 1.79 1.8 1.815 1.82 1.82 1.82;
1.86 1.8 1.81 1.82 1.825 1.825 1.825;
1.93 1.84 1.78 1.79 1.79 1.79 1.79;
2 1.9 1.825 1.76 1.75 1.75 1.75;
2.04 1.96 1.87 1.79 1.74 1.74 1.74;
2.16 2.05 1.92 1.84 1.79 1.79 1.79;
2.32 2.14 1.98 1.88 1.83 1.83 1.83;
2.44 2.26 2.1 1.97 1.88 1.88 1.88]; %(per hour)
[X,Y]=meshgrid(h,M);
T1=interp2(X,Y,Thrust,alt,mach);
TSFC1=interp2(X,Y,TSFC,alt,mach);

T_T0=T1/Thrust(1,1);
TSFC_TSFC0=TSFC1/TSFC(1,1);

T=T_T0*T0;
TSFC=TSFC_TSFC0*TSFC0;

end

function [rho,a] = atmos(h)
if exist('stdatmo','file') >= 2
    [rho,a] = stdatmo(h);
elseif exist('atmosisa','file') >= 2
    [T,~,~,rho] = atmosisa(h);
    a = sqrt(1.4*287.05287*T);
else
    link = 'http://www.mathworks.com/matlabcentral/fileexchange/28135';
    a='Standard atmosphere model on MATLAB path required';
    b=['<a href="' link '">STDATMO on file exchange</a>'];
    c=['<a href="' link '?download=true">STDATMO direct download</a>'];
    error('%s.\n%s\n%s',a,b,c)
end
end

