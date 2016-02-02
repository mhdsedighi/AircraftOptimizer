function [geo,lattice,struc,act]=part_rotation(act_num,geo,lattice,results,struc,state,act,wingno,partno,Raxle_local,alpha,hinge_portion,ref)

N_incements=3;
% struc.pointmass_exist=1;

% lattictype=1;
svertex_info=struc.vertex_info;
% vertex_info=lattice.vertex_info;
% sgeo=struc.geo;

% wing_parts=struc.geo.nelem(wingno);
wing_parts=geo.nelem(wingno);

if partno>1
    rail_partno=partno-1;
    rail_end=(svertex_info.X4(wingno,rail_partno,:)+svertex_info.X3(wingno,rail_partno,:))/2;
    rail_start=(svertex_info.X2(wingno,rail_partno,:)+svertex_info.X1(wingno,rail_partno,:))/2;
    rail_vec=rail_end-rail_start;
    course_max=norm(squeeze(rail_vec));
    rail_vec_uni=rail_vec/course_max;
    act_vec1=squeeze(rail_vec_uni);
    act_vec2=squeeze(svertex_info.X4(wingno,rail_partno,:)-svertex_info.X3(wingno,rail_partno,:));
    act_vec2=cross(act_vec1,act_vec2);
    act_vec2=cross(act_vec1,act_vec2);
    act_vec2=act_vec2/norm(act_vec2);
else
    act_vec1=[0;1;0];
    act_vec2=[-1;0;0];
end



% plotvec( rail_start,rail_vec,'r')

act_vec3=cross(act_vec1,act_vec2);

T=[act_vec1 act_vec2 act_vec3]';

Raxle_local=Raxle_local/norm(Raxle_local);
Raxle=(T'*Raxle_local')';

hinge_pos=squeeze(svertex_info.X1(wingno,partno,:)+hinge_portion*(svertex_info.X2(wingno,partno,:)-svertex_info.X1(wingno,partno,:)));
% 
plotvec( hinge_pos,act_vec1*3,'r')
plotvec( hinge_pos,act_vec2*3,'g')
plotvec( hinge_pos,act_vec3*3,'b')
% plotvec( hinge_pos,Raxle'*3,'k')


% svertex_info=struc.vertex_info;
% wing_parts=geo.nelem(wingno);
alpha_part=alpha/N_incements;

[npanel,~,~]=size(lattice.XYZ);
act(act_num).pos=hinge_pos';
% act(act_num).mass=act(act_num).mass_guess;
act(act_num).panel_consider=zeros(npanel,1);
act(act_num).wingno=wingno;
act(act_num).partno=partno;
act(act_num).sym=1;
for p=partno:geo.nelem(wingno)
    startindex1s=svertex_info.startindex1(wingno,p);
    endindex1s=svertex_info.endindex1(wingno,p);
    act(act_num).panel_consider(startindex1s:endindex1s)=1;
end

movement=linspace(0,alpha,N_incements);

% i=1;
% [results]=solver9_actuator(results,state,geo,lattice,act);
% Mx_=results.MOMENTS(1,1,1);
% My_=results.MOMENTS(1,1,2);
% Mz_=results.MOMENTS(1,1,3);
% M_act=T*[Mx_;My_;Mz_];
% Mx(i)=M_act(1);
% My(i)=M_act(2);
% Mz(i)=M_act(3);
F_g=zeros(3,N_incements);
M_g=F_g;
F_g_a=F_g;
M_g_a=F_g;
M_aero=F_g;
F_aero=F_g;
M_scalar=zeros(1,N_incements);
struc.pointmass=act;
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
    if i>1
        [geo,lattice]=part_rotation_geo(geo,lattice,struc,wingno,partno,Raxle,alpha_part,hinge_pos);
        [struc,act]=part_rotation_struc(struc,geo,act,wingno,partno,Raxle,alpha_part,hinge_pos);
        [struc]=massinertia(struc);
    end
    
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
end

F_act=F_aero+F_g+F_g_a;
M_act=M_aero+M_g+M_g_a;

for i=1:N_incements
    M_scalar(i)=dot(M_act(:,i),Raxle_local);
end

act(act_num).M=M_act;
act(act_num).F=F_act;
act(act_num).M_scalar=M_scalar;
act(act_num).movement=movement;



% act(act_num).energy=(trapz(movement,abs(M_act(1,:)))+trapz(movement,abs(M_act(2,:)))+trapz(movement,abs(M_act(3,:))));
% act(act_num).energy=trapz(abs(movement),abs(M_act(1,:)))+trapz(abs(movement),abs(M_act(2,:)))+trapz(abs(movement),abs(M_act(3,:)));
act(act_num).energy=trapz(abs(movement),abs(M_scalar));

end
function [geo,lattice]=part_rotation_geo(geo,lattice,struc,wingno,partno,Raxle,alpha,hinge_pos)
vertex_info=struc.vertex_info;
hinge_pos2=[1;-1;1].*squeeze(hinge_pos);


for p=partno:geo.nelem(wingno)
    
    startindex1=vertex_info.startindex1(wingno,p);
    endindex1=vertex_info.endindex1(wingno,p);
    
    
    
    
    
    
    %%Move to rotation centre
    l.V(:,:,1)=lattice.VORTEX(startindex1:endindex1,:,1)-hinge_pos(1);
    l.V(:,:,2)=lattice.VORTEX(startindex1:endindex1,:,2)-hinge_pos(2);
    l.V(:,:,3)=lattice.VORTEX(startindex1:endindex1,:,3)-hinge_pos(3);
    
    l.C(:,1)=lattice.COLLOC(startindex1:endindex1,1)-hinge_pos(1);
    l.C(:,2)=lattice.COLLOC(startindex1:endindex1,2)-hinge_pos(2);
    l.C(:,3)=lattice.COLLOC(startindex1:endindex1,3)-hinge_pos(3);
    
    l.XYZ(:,:,1)=lattice.XYZ(startindex1:endindex1,:,1)-hinge_pos(1);
    l.XYZ(:,:,2)=lattice.XYZ(startindex1:endindex1,:,2)-hinge_pos(2);
    l.XYZ(:,:,3)=lattice.XYZ(startindex1:endindex1,:,3)-hinge_pos(3);
    
    
    l.N(:,:,:)=lattice.N(startindex1:endindex1,:,:);
    
    
    [ai,bi,ci]=size(l.V);
    
    l2.C=trot4(Raxle,l.C,alpha)';
    l2.N=trot4(Raxle,l.N,alpha)';
    
    for i=2:(bi-1)
        A=squeeze(l.V(:,i,:))   ;
        B=trot4(Raxle,A,alpha)';
        l2.V(:,i,:)=B;
    end
    l2.V(:,1,:)=l.V(:,1,:);
    l2.V(:,bi,:)=l.V(:,bi,:);
    
    
    for i=1:5
        A=squeeze(l.XYZ(:,i,:)) ;
        B=trot4(Raxle,A,alpha)';
        l2.XYZ(:,i,:)=B;
    end
    
    
    %%Move back from rotation centre
    l2.V(:,:,1)=l2.V(:,:,1)+hinge_pos(1);
    l2.V(:,:,2)=l2.V(:,:,2)+hinge_pos(2);
    l2.V(:,:,3)=l2.V(:,:,3)+hinge_pos(3);
    
    l2.C(:,1)=l2.C(:,1)+hinge_pos(1);
    l2.C(:,2)=l2.C(:,2)+hinge_pos(2);
    l2.C(:,3)=l2.C(:,3)+hinge_pos(3);
    
    l2.XYZ(:,:,1)=l2.XYZ(:,:,1)+hinge_pos(1);
    l2.XYZ(:,:,2)=l2.XYZ(:,:,2)+hinge_pos(2);
    l2.XYZ(:,:,3)=l2.XYZ(:,:,3)+hinge_pos(3);
    
    
    
    lattice.VORTEX(startindex1:endindex1,:,:)=l2.V;
    lattice.COLLOC(startindex1:endindex1,:,:)=l2.C;
    lattice.XYZ(startindex1:endindex1,:,:)=l2.XYZ;
    lattice.N(startindex1:endindex1,:,:)=l2.N;
    %     struc.XYZ(startindex1:endindex1,:,:)=l2.XYZ;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if geo.symetric(wingno)
        startindex2=vertex_info.startindex2(wingno,p);
        endindex2=vertex_info.endindex2(wingno,p);
        
        
        %%Move to rotation centre
        l.V(:,:,1)=lattice.VORTEX(startindex2:endindex2,:,1)-hinge_pos2(1);
        l.V(:,:,2)=lattice.VORTEX(startindex2:endindex2,:,2)-hinge_pos2(2);
        l.V(:,:,3)=lattice.VORTEX(startindex2:endindex2,:,3)-hinge_pos2(3);
        
        l.C(:,1)=lattice.COLLOC(startindex2:endindex2,1)-hinge_pos2(1);
        l.C(:,2)=lattice.COLLOC(startindex2:endindex2,2)-hinge_pos2(2);
        l.C(:,3)=lattice.COLLOC(startindex2:endindex2,3)-hinge_pos2(3);
        
        l.XYZ(:,:,1)=lattice.XYZ(startindex2:endindex2,:,1)-hinge_pos2(1);
        l.XYZ(:,:,2)=lattice.XYZ(startindex2:endindex2,:,2)-hinge_pos2(2);
        l.XYZ(:,:,3)=lattice.XYZ(startindex2:endindex2,:,3)-hinge_pos2(3);
        
        l.N(:,:,:)=lattice.N(startindex2:endindex2,:,:);
        
        
        
        % [ai bi ci]=size(l.V);
        
        l2.C=trot4(Raxle,l.C,-alpha)';
        l2.N=trot4(Raxle,l.N,-alpha)';
        
        for i=2:(bi-1)
            A=squeeze(l.V(:,i,:))   ;
            B=trot4(Raxle,A,-alpha)';
            l2.V(:,i,:)=B;
        end
        
        
        l2.V(:,1,:)=l.V(:,1,:);
        l2.V(:,bi,:)=l.V(:,bi,:);
        
        
        for i=1:5
            A=squeeze(l.XYZ(:,i,:)) ;
            B=trot4(Raxle,A,-alpha)';
            l2.XYZ(:,i,:)=B;
        end
        
        %%Move back from rotation centre
        l2.V(:,:,1)=l2.V(:,:,1)+hinge_pos2(1);
        l2.V(:,:,2)=l2.V(:,:,2)+hinge_pos2(2);
        l2.V(:,:,3)=l2.V(:,:,3)+hinge_pos2(3);
        
        l2.C(:,1)=l2.C(:,1)+hinge_pos2(1);
        l2.C(:,2)=l2.C(:,2)+hinge_pos2(2);
        l2.C(:,3)=l2.C(:,3)+hinge_pos2(3);
        
        l2.XYZ(:,:,1)=l2.XYZ(:,:,1)+hinge_pos2(1);
        l2.XYZ(:,:,2)=l2.XYZ(:,:,2)+hinge_pos2(2);
        l2.XYZ(:,:,3)=l2.XYZ(:,:,3)+hinge_pos2(3);
        
        
        lattice.VORTEX(startindex2:endindex2,:,:)=l2.V;
        lattice.COLLOC(startindex2:endindex2,:,:)=l2.C;
        lattice.XYZ(startindex2:endindex2,:,:)=l2.XYZ;
        lattice.N(startindex2:endindex2,:,:)=l2.N;
        %         struc.XYZ(startindex2:endindex2,:,:)=l2.XYZ;
    end
    
    
end


% [struc]=part_rotation_struc(wingno,partno,struc,Raxle,alpha,hinge_pos);


end
function [struc,act]=part_rotation_struc(struc,geo,act,wingno,partno,Raxle,alpha,hinge_pos)

% geo=struc.geo;
svertex_info=struc.vertex_info;
% wing_parts=struc.geo.nelem(wingno);
hinge_pos2=[1;-1;1].*squeeze(hinge_pos);


for i=1:length(act)
    if ~isempty(act(i).wingno)
        if act(i).wingno==wingno && act(i).partno>partno
            pos_old=act(i).pos-hinge_pos';
            pos_new=trot4(Raxle,pos_old,alpha)';
            act(i).pos=pos_new+hinge_pos';
        end
    end
end
struc.pointmass=act;

wing_parts=geo.nelem(wingno);
for p=partno:wing_parts
    
    startindex1s=svertex_info.startindex1(wingno,p);
    endindex1s=svertex_info.endindex1(wingno,p);
    
    %     hinge_pos=svertex_info.X1(wingno,partno,:)+hinge_portion*(svertex_info.X2(wingno,partno,:)-svertex_info.X1(wingno,partno,:));
    
    
    
    %%Move to rotation centre
    
    ls.XYZ(:,:,1)=struc.XYZ(startindex1s:endindex1s,:,1)-hinge_pos(1);
    ls.XYZ(:,:,2)=struc.XYZ(startindex1s:endindex1s,:,2)-hinge_pos(2);
    ls.XYZ(:,:,3)=struc.XYZ(startindex1s:endindex1s,:,3)-hinge_pos(3);
    
    
    
    
    for i=1:5
        A=squeeze(ls.XYZ(:,i,:)) ;
        B=trot4(Raxle,A,alpha)';
        l2s.XYZ(:,i,:)=B;
    end
    
    
    
    
    %%Move back from rotation centre
    l2s.XYZ(:,:,1)=l2s.XYZ(:,:,1)+hinge_pos(1);
    l2s.XYZ(:,:,2)=l2s.XYZ(:,:,2)+hinge_pos(2);
    l2s.XYZ(:,:,3)=l2s.XYZ(:,:,3)+hinge_pos(3);
    
    
    struc.XYZ(startindex1s:endindex1s,:,:)=l2s.XYZ;
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if struc.geo.symetric(wingno)
        
        startindex2s=svertex_info.startindex2(wingno,p);
        endindex2s=svertex_info.endindex2(wingno,p);
        %         hinge_pos2=[1;-1;1].*squeeze(hinge_pos);
        
        %%Move to rotation centre
        ls.XYZ(:,:,1)=struc.XYZ(startindex2s:endindex2s,:,1)-hinge_pos2(1);
        ls.XYZ(:,:,2)=struc.XYZ(startindex2s:endindex2s,:,2)-hinge_pos2(2);
        ls.XYZ(:,:,3)=struc.XYZ(startindex2s:endindex2s,:,3)-hinge_pos2(3);
        
        
        
        % [ai bi ci]=size(l.V);
        
        
        
        
        for i=1:5
            A=squeeze(ls.XYZ(:,i,:)) ;
            B=trot4(Raxle,A,-alpha)';
            l2s.XYZ(:,i,:)=B;
        end
        
        %%Move back from rotation centre
        l2s.XYZ(:,:,1)=l2s.XYZ(:,:,1)+hinge_pos2(1);
        l2s.XYZ(:,:,2)=l2s.XYZ(:,:,2)+hinge_pos2(2);
        l2s.XYZ(:,:,3)=l2s.XYZ(:,:,3)+hinge_pos2(3);
        
        
        
        struc.XYZ(startindex2s:endindex2s,:,:)=l2s.XYZ;
    end
    
    
end


end

%function wingrotation
function[p2]=trot4(hinge,p,alpha)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TROT: Auxillary rotation function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% rotates point p around hinge alpha rads.%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ref: 	Råde, Westergren, BETA 4th ed,
%			studentlitteratur, 1998
%			pp:107-108
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: 	Tomas Melin, KTH,Department of%
% 				aeronautics, Copyright 2000
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Context:	Auxillary function for
%				TORNADO.
% Called by: setrudder, normals
% Calls:		norm (MATLAB std fcn)
%				sin			"
%				cos			"
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% HELP:		Hinge=vector around rotation
%						takes place.
%				p=point to be rotated
%				alpha=radians of rotation
%				3D-workspace
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
a=hinge(1);
b=hinge(2);
c=hinge(3);

rho=sqrt(a^2+b^2);
r=sqrt(a^2+b^2+c^2);

if r==0
    cost=0;
    sint=1;
else
    cost=c/r;
    sint=rho/r;
end

if rho==0
    cosf=0;
    sinf=1;
else
    cosf=a/rho;
    sinf=b/rho;
end

cosa=cos(alpha);
sina=sin(alpha);

RZF=[[cosf -sinf 0];[sinf cosf 0];[0 0 1]];
RYT=[[cost 0 sint];[0 1 0];[-sint 0 cost]];
RZA=[[cosa -sina 0];[sina cosa 0];[0 0 1]];
RYMT=[[cost 0 -sint];[0 1 0];[sint 0 cost]];
RZMF=[[cosf sinf 0];[-sinf cosf 0];[0 0 1]];

P=RZF*RYT*RZA*RYMT*RZMF;
p2=P*p';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end%function wingrotation
