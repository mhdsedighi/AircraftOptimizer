function state=change_state(state,h,v)

state.ALT=h;
state.AS=v;
[rho,a,~,mu]=ISAtmosphere(state.ALT);     %Calling International Standard atmosphere.
state.rho=rho;
state.Mach=state.AS/a;
state.q=0.5*(state.rho)*(state.AS)^2;
state.a=a;
state.mu=mu;

end