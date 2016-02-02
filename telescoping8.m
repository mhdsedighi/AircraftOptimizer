function [geo,lattice,struc,act]=telescoping8(act_num,geo,ref,lattice,results,state,struc,act,wingno,partno,course_portion)
%%%%%%%%%%%%%%%%%%%%%%%
N_incements=5;
struc.pointmass_exist=1;
% vertex_info=lattice.vertex_info;
lattictype=1;
svertex_info=struc.vertex_info;
sgeo=struc.geo;

% wing_parts=sgeo.nelem(wingno);
wing_parts=struc.geo.nelem(wingno);

rail_partno=partno-1;
rail_end=(svertex_info.X4(wingno,rail_partno,:)+svertex_info.X3(wingno,rail_partno,:))/2;
rail_start=(svertex_info.X2(wingno,rail_partno,:)+svertex_info.X1(wingno,rail_partno,:))/2;
rail_vec=rail_end-rail_start;
course_max=norm(squeeze(rail_vec));
rail_vec_uni=rail_vec/course_max;

% plotvec( rail_start,rail_vec,'r')

act_vec1=squeeze(rail_vec_uni);
act_vec2=squeeze(svertex_info.X4(wingno,rail_partno,:)-svertex_info.X3(wingno,rail_partno,:));
act_vec2=cross(act_vec1,act_vec2);
act_vec2=cross(act_vec1,act_vec2);
act_vec2=act_vec2/norm(act_vec2);
act_vec3=cross(act_vec1,act_vec2);

T=[act_vec1 act_vec2 act_vec3];

plotvec( rail_end,act_vec1,'r')
plotvec( rail_end,act_vec2,'g')
plotvec( rail_end,act_vec3,'b')
% hold on
% plot3(rail_end(1),rail_end(2),rail_end(3),'k*')
% plot3(rail_start(1),rail_start(2),rail_start(3),'r*')

[npanel,~,~]=size(lattice.XYZ);

act(act_num).mass=act(act_num).mass;
act(act_num).wingno=wingno;
act(act_num).partno=partno;
act(act_num).sym=1;
act(act_num).pos=squeeze(rail_end)';
act(act_num).panel_consider=zeros(npanel,1);
% act.movement=course_max*course_portion;
for p=partno:geo.nelem(wingno)
    startindex1s=svertex_info.startindex1(wingno,p);
    endindex1s=svertex_info.endindex1(wingno,p);
    act(act_num).panel_consider(startindex1s:endindex1s)=1;
end

%%%%%%%%%%%%%%%%
course_incerement=linspace(0,1,N_incements);
movement=zeros(1,N_incements);
F_g=zeros(3,N_incements);
M_g=F_g;
F_g_a=F_g;
M_g_a=F_g;
M_aero=F_g;
F_aero=F_g;
geo_backup=geo;
lattice_backup=lattice;
struc_backup=struc;
results_backup=results;

m_actuator=zeros(wing_parts,1);
pos_actuator=zeros(wing_parts,3);
for k=1:length(act)
    if ~isempty(act(k).wingno)
        if act(k).wingno==wingno && act(k).partno>partno
            m_actuator(partno)=act(k).mass;
        end
    end
end

for i=1:N_incements
    course_portion1=course_portion*course_incerement(i);
    %         course_vec=rail_vec-(1-course_portion)*sgeo.b(wingno,partno);
    course_vec=course_portion1*sgeo.b(wingno,partno)*rail_vec_uni;
    %     plotvec( [0 0 0]',course_vec,'r')
    %     course_vec=course_incerement(i)*course_portion*sgeo.b(wingno,partno)*rail_vec;
    %     course_vec=[0 0 0]';
    
    movement(i)=course_portion1*sgeo.b(wingno,partno);
    
    [geo,struc]=telescoping_geo(geo,struc,act,wingno,partno,course_portion1,svertex_info,course_vec);
    [lattice,~]=fLattice_setup2(geo,state,lattictype,ref);
    [results]=solver9_actuator(results,state,geo,lattice,act(act_num),ref);
    
    
    
    Mx_=results.MOMENTS(1,1,1);
    My_=results.MOMENTS(1,1,2);
    Mz_=results.MOMENTS(1,1,3);
    Fx_=results.FORCE(1,1,1);
    Fy_=results.FORCE(1,1,2);
    Fz_=results.FORCE(1,1,3);
    M_aero(:,i)=T*[Mx_;My_;Mz_];
    F_aero(:,i)=T*[Fx_;Fy_;Fz_];
    
    for k=1:length(act)
        if ~isempty(act(k).wingno)
            if act(k).wingno==wingno && act(k).partno>partno
                pos_actuator(partno,:)=act(k).pos;
            end
        end
    end
    
    for counter=partno:wing_parts
        g_moment_arm=T*(squeeze(struc.centroid_asym(wingno,counter,:))-act(act_num).pos');
        F_g(:,i)=F_g(:,i)+T*struc.mass(wingno,counter)*[0;0;state.g];
        M_g(:,i)=M_g(:,i)+cross(g_moment_arm,F_g(:,i));
    end
    for counter=partno+1:wing_parts
        g_moment_arm_a=T*(pos_actuator(counter,:)-act(act_num).pos)';
        F_g_a(:,i)=F_g_a(:,i)+T*m_actuator(counter)*[0;0;state.g];
        M_g_a(:,i)=M_g_a(:,i)+cross(g_moment_arm_a,F_g_a(:,i));
    end
    %     FX(i)=FX_(1);
    %     FY(i)=FY_(1);
    %     FZ(i)=FZ_(1);
    %     F_body=[Fx_;Fy_;Fz_];
    %     F_act=T*F_body;
    %     Fx(i)=F_act(1);
    %     Fy(i)=F_act(2);
    %     Fz(i)=F_act(3);
    %     Cn=results.MOMENTS(1,1,3)
    %     [geo,lattice,struc]=telescoping_geo(geo,lattice,struc,wingno,partno,1/course_portion1,svertex_info,-course_vec);
    %     [lattice,ref]=fLattice_setup2(geo,state,lattictype);
    if i<N_incements
        geo=geo_backup;
        lattice=lattice_backup;
        struc=struc_backup;
        results=results_backup;
    end
end

% course_vec=course_portion*sgeo.b(wingno,partno)*rail_vec_uni;
% [geo,struc]=telescoping_geo(geo,struc,wingno,partno,course_portion,svertex_info,course_vec);
% [lattice,~]=fLattice_setup2(geo,state,lattictype);

F_act=F_aero+F_g+F_g_a;
M_act=M_aero+M_g+M_g_a;

% course_vec=course_portion*sgeo.b(wingno,partno)*rail_vec;
act(act_num).M(:,:)=M_act;
act(act_num).F(:,:)=F_act;
act(act_num).F_aero(:,:)=F_aero;
act(act_num).F_g(:,:)=F_g;
act(act_num).movement(:)=movement;
% act(act_num).energy=trapz(movement,abs(F_act(1,:)))+trapz(movement,abs(F_act(2,:)))+trapz(movement,abs(F_act(3,:)));
act(act_num).energy=0.5*abs(trapz(movement,abs(F_act(1,:))));
% act.enegry_telescoping=act.telescoping.cf*(trapz(movement,abs(Fy))+trapz(movement,abs(Fz)));

end
function [geo,struc]=telescoping_geo(geo,struc,act,wingno,partno,course_portion,svertex_info,course_vec)

% vertex_info=lattice.vertex_info;
% svertex_info=struc.vertex_info;
% sgeo=struc.geo;
%
% % wing_parts=sgeo.nelem(wingno);
% wing_parts=geo.nelem(wingno);
%
% rail_partno=partno-1;
% rail_end=(svertex_info.X4_f(wingno,rail_partno,:)+svertex_info.X3(wingno,rail_partno,:))/2;
% rail_start=(svertex_info.X2_f(wingno,rail_partno,:)+svertex_info.X1(wingno,rail_partno,:))/2;
% rail_vec=rail_end-rail_start;
% course_max=norm(squeeze(rail_vec));
% rail_vec=rail_vec/course_max;
%
% % plotvec( rail_start,rail_vec,'r')
%
%
% course_vec=course_portion*sgeo.b(wingno,partno)*rail_vec;
% [npanel,~,~]=size(lattice.XYZ);
% act.pos=squeeze(rail_end)';
% act.panel_consider=zeros(npanel,1);
% act.movement=course_max*course_portion;

for i=1:length(act)
    if ~isempty(act(i).wingno)
        if act(i).wingno==wingno && act(i).partno>partno
            pos_new=act(i).pos-course_vec;
            act(i).pos=pos_new';
        end
    end
end
   struc.pointmass=act;

for p=partno:geo.nelem(wingno)
    
    startindex1s=svertex_info.startindex1(wingno,p);
    endindex1s=svertex_info.endindex1(wingno,p);
    
    %     act.panel_consider(startindex1s:endindex1s)=1;
    
    %%Move to rotation centre
    
    l.XYZ(:,:,1)=struc.XYZ(startindex1s:endindex1s,:,1)-course_vec(1);
    l.XYZ(:,:,2)=struc.XYZ(startindex1s:endindex1s,:,2)-course_vec(2);
    l.XYZ(:,:,3)=struc.XYZ(startindex1s:endindex1s,:,3)-course_vec(3);
    
    
    struc.XYZ(startindex1s:endindex1s,:,:)=l.XYZ;
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if geo.symetric(wingno)
        startindex2s=svertex_info.startindex2(wingno,p);
        endindex2s=svertex_info.endindex2(wingno,p);
        course_vec2=[1;-1;1].*squeeze(course_vec);
        
        
        %%Move to rotation centre
        l.XYZ(:,:,1)=struc.XYZ(startindex2s:endindex2s,:,1)-course_vec2(1);
        l.XYZ(:,:,2)=struc.XYZ(startindex2s:endindex2s,:,2)-course_vec2(2);
        l.XYZ(:,:,3)=struc.XYZ(startindex2s:endindex2s,:,3)-course_vec2(3);
        
        
        struc.XYZ(startindex2s:endindex2s,:,:)=l.XYZ;
    end
    
    
end
%%%%%%%


if course_portion==1
    
    
    geo.b(wingno,partno)= geo.b(wingno,partno)*(1-0.999999);
    geo.T(wingno,partno)=geo.T(wingno,partno)/(1-0.999999);
    
    %     geo.nelem(wingno)=geo.nelem(wingno)-1;
    %     geo.b=ommit_section(geo.b,wingno,partno);
    %     geo.dihed=ommit_section(geo.dihed,wingno,partno);
    %     geo.T=ommit_section(geo.T,wingno,partno);
    %     geo.SW=ommit_section(geo.SW,wingno,partno);
    %     geo.TW=ommit_section(geo.TW,wingno,partno);
    %     geo.nx=ommit_section(geo.nx,wingno,partno);
    %     geo.ny=ommit_section(geo.ny,wingno,partno);
    %     geo.flapped=ommit_section(geo.flapped,wingno,partno);
    %     geo.fnx=ommit_section(geo.fnx,wingno,partno);
    %     geo.fsym=ommit_section(geo.fsym,wingno,partno);
    %     geo.fc=ommit_section(geo.fc,wingno,partno);
    %     geo.flap_vector=ommit_section(geo.flap_vector,wingno,partno);
    %     geo.foil(:,:,1)=geo.foil(:,:,1);
    
elseif course_portion<1
    geo.b(wingno,partno)= geo.b(wingno,partno)*(1-course_portion);
    geo.T(wingno,partno)=geo.T(wingno,partno)*(1-course_portion);
    
end

% b=sum(geo.b(wingno,:));




end%function wingrotation