close all
% b1=[i_Xwing' i_Zwing' i_bfrac1' i_sweep_angle' i_twist_angle' i_taper'];
% b2=i_actmass;
% b3=[i_rot_angle' i_Raxle_local];
% b4=[i_alpha' i_elevator_angle' i_throttle' i_AS'];

clear natij aa_trim_code1 aa_morph_code1 aa_design_code1 aa_actmass_code1
input=x;
aa_design_code1(1:N_design)=input(1:N_design);
aa_actmass_code1(1:N_act)=input(N_design+1:N_design+N_act);
inputmat=vector2matrix(input(N_design+N_act+1:end),N_condition,N_trim+N_morph);
for j=1:N_condition
    aa_morph_code1(1:N_morph,j)=inputmat(j,1:N_morph);
    aa_trim_code1(1:N_trim,j)=inputmat(j,N_morph+1:N_morph+N_trim);
end

% aa_design_code1(4)=aa_design_code1(4)*180/pi;
% aa_design_code1(5)=aa_design_code1(5)*180/pi;
% aa_morph_code1(1,:)=aa_morph_code1(1,:)*180/pi;
aa_trim_code1(1,:)=aa_trim_code1(1,:)*180/pi;
aa_trim_code1(2,:)=aa_trim_code1(2,:)*180/pi;

% natij_des(1)=aa_design_code1(1)
% natij_des(2)=aa_design_code1(2)
% natij_des(3)=aa_design_code1(3)
% natij_des(4)=aa_design_code1(4)*180/pi
% natij_des(5)=aa_design_code1(5)*180/pi
% natij_des(6)=aa_design_code1(6)
% natij_des(7)=actmass_code1(1)
% natij_des(8)=morph_code1(1)
% natij_des(9)=morph_code1(2)
% natij_des(10)=morph_code1(3)
% natij_des(11)=morph_code1(4)
% natij_des(12)=trim_code1(1)
% natij_des(13)=trim_code1(1)
% natij_des(14)=trim_code1(1)
% natij_des(15)=trim_code1(1)



% for i=1:N_condition
%     energy(i)=ans_act(i).energy;
% end
% max_energy=max(energy)


for j=1:N_condition

aa_natij(1,j)=ans_perf(j).LOD;
aa_natij(2,j)=ans_perf(j).Range;
aa_natij(3,j)=ans_perf(j).Endurance;
aa_natij(4,j)=abs(ans_perf(j).ROC);
aa_natij(5,j)=ans_perf(j).stability.SM;
aa_natij(6,j)=nan;
aa_natij(7,j)=ans_perf(j).stability.w_SP;
aa_natij(7,j)=ans_perf(j).stability.z_SP;
aa_natij(9,j)=ans_perf(j).stability.w_PH;
aa_natij(10,j)=ans_perf(j).stability.z_PH;
aa_natij(11,j)=abs(ans_perf(j).stability.w_D);
aa_natij(12,j)=abs(ans_perf(j).stability.z_D);
aa_natij(13,j)=ans_perf(j).stability.T_SR;
aa_natij(13,j)=ans_perf(j).stability.T_R;

end


for i=1:N_condition
    figure
    subplot(2,2,1)
    plot_plane_lokht(body,ans_geo(i),ans_struc(i))
    view(-90,0)
    subplot(2,2,2)
    plot_plane_lokht(body,ans_geo(i),ans_struc(i))
    view(0,0)
    subplot(2,2,3)
    plot_plane_lokht(body,ans_geo(i),ans_struc(i))
    view(90,90)
    subplot(2,2,4)
    plot_plane(body,ans_geo(i),ans_struc(i))
    view(-32,12)
end


for i=1:N_condition
%     geometryplot(ans_lattice(1),ans_geo(i),ans_ref(i))
    
end


% 
% % 
% figure
% subplot(2,3,1)
% title('No Morphing')
% plot_plane(body,geo,struc)
% view(-70,22)
% 
% subplot(2,3,2)
% title('Mode 1')
% plot_plane(body,ans_geo(1),ans_struc(1))
% view(-70,22)
% 
% subplot(2,3,3)
% title('Mode 2')
% plot_plane(body,ans_geo(2),ans_struc(2))
% view(-70,22)
% 
% subplot(2,3,4)
% title('Mode 3')
% plot_plane(body,ans_geo(3),ans_struc(3))
% view(-70,22)
% 
% subplot(2,3,5)
% title('Mode 4')
% plot_plane(body,ans_geo(4),ans_struc(4))
% view(-70,22)
% 
% subplot(2,3,6)
% title('Mode 5')
% plot_plane(body,ans_geo(5),ans_struc(5))
% view(-70,22)