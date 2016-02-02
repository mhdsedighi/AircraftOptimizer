function [ act_out ] = actuator_sizing(act)

act_out=act;
% for act_num=1:length(act)
% act_out(act_num).mass_guess=act(act_num).energy/5/1000/0.1;
% end



% 0.42471653925086056, 22.767145586385844

% act_out.mass_guess=act.energy/10/135;
[~,N]=size(act(1).M_scalar);
M=zeros(1,N);
for i=1:N
    M(i)=abs(act(1).M_scalar(i));
end

% act_out.mass=max(M)/1000;
act_out.mass=max(M)/1000;

end

