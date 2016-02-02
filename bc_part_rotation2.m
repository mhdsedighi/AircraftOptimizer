function [geo,lattice,struc,act]=part_rotation(geo,lattice,results,struc,state,act,wingno,partno,Raxle_local,alpha,hinge_portion)

% lattictype=1;
svertex_info=struc.vertex_info;
% vertex_info=lattice.vertex_info;
% sgeo=struc.geo;

% wing_parts=sgeo.nelem(wingno);
% wing_parts=geo.nelem(wingno);

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

act_vec3=cross(act_vec2,act_vec1);

T=[act_vec1 act_vec2 act_vec3];

Raxle=(T'*Raxle_local')';

hinge_pos=squeeze(svertex_info.X1(wingno,partno,:)+hinge_portion*(svertex_info.X2(wingno,partno,:)-svertex_info.X1(wingno,partno,:)));

plotvec( hinge_pos,act_vec1,'r')
plotvec( hinge_pos,act_vec2,'g')
plotvec( hinge_pos,act_vec3,'b')


% svertex_info=struc.vertex_info;
% wing_parts=geo.nelem(wingno);
alpha_part=alpha;

[npanel,~,~]=size(lattice.XYZ);
act.actuation_ref_point=hinge_pos';
act.panel_consider=zeros(npanel,1);
for p=partno:geo.nelem(wingno)
    startindex1s=svertex_info.startindex1(wingno,p);
    endindex1s=svertex_info.endindex1(wingno,p);
    act.panel_consider(startindex1s:endindex1s)=1;
end
for i=1:1
    [geo,lattice,struc]=part_rotation_geo(geo,lattice,struc,wingno,partno,Raxle,alpha,hinge_pos);
%     [struc]=part_rotation_struc(wingno,partno,struc,Raxle,alpha_part,hinge_pos);
    [results]=solver9_actuator(results,state,geo,lattice,act);
    Mx_=results.MOMENTS(1,:,1);
    My_=results.MOMENTS(1,:,2);
    Mz_=results.MOMENTS(1,:,3);
end



end
function [geo,lattice,struc]=part_rotation_geo(geo,lattice,struc,wingno,partno,Raxle,alpha,hinge_pos)
vertex_info=lattice.vertex_info;



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
    struc.XYZ(startindex1s:endindex1s,:,:)=l2s.XYZ;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if geo.symetric(wingno)
        startindex2=vertex_info.startindex2(wingno,p);
        endindex2=vertex_info.endindex2(wingno,p);
        %     hinge_pos=[1;-1;1].*squeeze(vertex_info.X3(wingno,partno,:));
        hinge_pos=[1;-1;1].*squeeze(hinge_pos);
        
        %%Move to rotation centre
        l.V(:,:,1)=lattice.VORTEX(startindex2:endindex2,:,1)-hinge_pos(1);
        l.V(:,:,2)=lattice.VORTEX(startindex2:endindex2,:,2)-hinge_pos(2);
        l.V(:,:,3)=lattice.VORTEX(startindex2:endindex2,:,3)-hinge_pos(3);
        
        l.C(:,1)=lattice.COLLOC(startindex2:endindex2,1)-hinge_pos(1);
        l.C(:,2)=lattice.COLLOC(startindex2:endindex2,2)-hinge_pos(2);
        l.C(:,3)=lattice.COLLOC(startindex2:endindex2,3)-hinge_pos(3);
        
        l.XYZ(:,:,1)=lattice.XYZ(startindex2:endindex2,:,1)-hinge_pos(1);
        l.XYZ(:,:,2)=lattice.XYZ(startindex2:endindex2,:,2)-hinge_pos(2);
        l.XYZ(:,:,3)=lattice.XYZ(startindex2:endindex2,:,3)-hinge_pos(3);
        
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
        l2.V(:,:,1)=l2.V(:,:,1)+hinge_pos(1);
        l2.V(:,:,2)=l2.V(:,:,2)+hinge_pos(2);
        l2.V(:,:,3)=l2.V(:,:,3)+hinge_pos(3);
        
        l2.C(:,1)=l2.C(:,1)+hinge_pos(1);
        l2.C(:,2)=l2.C(:,2)+hinge_pos(2);
        l2.C(:,3)=l2.C(:,3)+hinge_pos(3);
        
        l2.XYZ(:,:,1)=l2.XYZ(:,:,1)+hinge_pos(1);
        l2.XYZ(:,:,2)=l2.XYZ(:,:,2)+hinge_pos(2);
        l2.XYZ(:,:,3)=l2.XYZ(:,:,3)+hinge_pos(3);
        
        
        lattice.VORTEX(startindex2:endindex2,:,:)=l2.V;
        lattice.COLLOC(startindex2:endindex2,:,:)=l2.C;
        lattice.XYZ(startindex2:endindex2,:,:)=l2.XYZ;
        lattice.N(startindex2:endindex2,:,:)=l2.N;
        struc.XYZ(startindex1s:endindex1s,:,:)=l2s.XYZ;
    end
    
    
end


% [struc]=part_rotation_struc(wingno,partno,struc,Raxle,alpha,hinge_pos);
struc.XYZ(startindex2s:endindex2s,:,:)=l2s.XYZ;

end
function [struc]=part_rotation_struc(wingno,partno,struc,Raxle,alpha,hinge_pos)

% geo=struc.geo;
svertex_info=struc.vertex_info;
wing_parts=struc.geo.nelem(wingno);

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
        hinge_pos=[1;-1;1].*squeeze(hinge_pos);
        
        %%Move to rotation centre
        ls.XYZ(:,:,1)=struc.XYZ(startindex2s:endindex2s,:,1)-hinge_pos(1);
        ls.XYZ(:,:,2)=struc.XYZ(startindex2s:endindex2s,:,2)-hinge_pos(2);
        ls.XYZ(:,:,3)=struc.XYZ(startindex2s:endindex2s,:,3)-hinge_pos(3);
        
        
        
        % [ai bi ci]=size(l.V);
        
        for i=1:5
            A=squeeze(ls.XYZ(:,i,:)) ;
            B=trot4(Raxle,A,-alpha)';
            l2s.XYZ(:,i,:)=B;
        end
        
        %%Move back from rotation centre
        l2s.XYZ(:,:,1)=l2s.XYZ(:,:,1)+hinge_pos(1);
        l2s.XYZ(:,:,2)=l2s.XYZ(:,:,2)+hinge_pos(2);
        l2s.XYZ(:,:,3)=l2s.XYZ(:,:,3)+hinge_pos(3);
        
        
        
        
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
% ref: 	R�de, Westergren, BETA 4th ed,
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