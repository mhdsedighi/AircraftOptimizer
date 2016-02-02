function [struc]=part_rotation_struc(wingno,partno,struc,Raxle,alpha,hinge_portion)

geo=struc.geo;
svertex_info=struc.vertex_info;
wing_parts=geo.nelem(wingno);

for p=partno:wing_parts
    
    
    startindex1s=svertex_info.startindex1(wingno,p);
    endindex1s=svertex_info.endindex1(wingno,p);
    
    hinge_pos=svertex_info.X1(wingno,partno,:)+hinge_portion*(svertex_info.X2(wingno,partno,:)-svertex_info.X1(wingno,partno,:));
    
    
    
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
    
    if geo.symetric(wingno)
        
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