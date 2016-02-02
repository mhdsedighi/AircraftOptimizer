function [lattice,geo,struc]=part_rotation(wingno,partno,geo,lattice,struc,Raxle,alpha,hinge_portion)


vertex_info=lattice.vertex_info;
wing_parts=geo.nelem(wingno);

for p=partno:wing_parts
    
    startindex1=vertex_info.startindex1(wingno,p);
    endindex1=vertex_info.endindex1(wingno,p);
    
    hinge_pos=vertex_info.X1(wingno,partno,:)+hinge_portion*(vertex_info.X2(wingno,partno,:)-vertex_info.X1(wingno,partno,:));
    
    
    
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
    ls.XYZ(:,:,1)=struc.XYZ(startindex1:endindex1,:,1)-hinge_pos(1);
    ls.XYZ(:,:,2)=struc.XYZ(startindex1:endindex1,:,2)-hinge_pos(2);
    ls.XYZ(:,:,3)=struc.XYZ(startindex1:endindex1,:,3)-hinge_pos(3);
    
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
    for i=1:5
        A=squeeze(ls.XYZ(:,i,:)) ;
        B=trot4(Raxle,A,alpha)';
        l2s.XYZ(:,i,:)=B;
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
    l2s.XYZ(:,:,1)=l2s.XYZ(:,:,1)+hinge_pos(1);
    l2s.XYZ(:,:,2)=l2s.XYZ(:,:,2)+hinge_pos(2);
    l2s.XYZ(:,:,3)=l2s.XYZ(:,:,3)+hinge_pos(3);
    
    
    lattice.VORTEX(startindex1:endindex1,:,:)=l2.V;
    lattice.COLLOC(startindex1:endindex1,:,:)=l2.C;
    lattice.XYZ(startindex1:endindex1,:,:)=l2.XYZ;
    struc.XYZ(startindex1:endindex1,:,:)=l2s.XYZ;
    lattice.N(startindex1:endindex1,:,:)=l2.N;
    
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
        ls.XYZ(:,:,1)=struc.XYZ(startindex2:endindex2,:,1)-hinge_pos(1);
        ls.XYZ(:,:,2)=struc.XYZ(startindex2:endindex2,:,2)-hinge_pos(2);
        ls.XYZ(:,:,3)=struc.XYZ(startindex2:endindex2,:,3)-hinge_pos(3);
        
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
        for i=1:5
            A=squeeze(ls.XYZ(:,i,:)) ;
            B=trot4(Raxle,A,-alpha)';
            l2s.XYZ(:,i,:)=B;
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
        l2s.XYZ(:,:,1)=l2.XYZ(:,:,1)+hinge_pos(1);
        l2s.XYZ(:,:,2)=l2.XYZ(:,:,2)+hinge_pos(2);
        l2s.XYZ(:,:,3)=l2.XYZ(:,:,3)+hinge_pos(3);
        
        
        lattice.VORTEX(startindex2:endindex2,:,:)=l2.V;
        lattice.COLLOC(startindex2:endindex2,:,:)=l2.C;
        lattice.XYZ(startindex2:endindex2,:,:)=l2.XYZ;
        struc.XYZ(startindex2:endindex2,:,:)=l2s.XYZ;
        lattice.N(startindex2:endindex2,:,:)=l2.N;
    end
    
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% preserving connection in joints
% if isattached
%     wing_parts=geo.nelem(wingno);
%     for i=partno+1:wing_parts
%
%         startindex1=vertex_info.startindex1(wingno,i);
%         endindex1=vertex_info.endindex1(wingno,i);
%
%         pos_now=[lattice.XYZ(vertex_info.K1(wingno,i),1,1) lattice.XYZ(vertex_info.K1(wingno,i),1,2) lattice.XYZ(vertex_info.K1(wingno,i),1,3)];
%         pos_target=[lattice.XYZ(vertex_info.K3(wingno,i-1),2,1) lattice.XYZ(vertex_info.K3(wingno,i-1),2,2) lattice.XYZ(vertex_info.K3(wingno,i-1),2,3)];
%
%         delta_pos=pos_now-pos_target;
%         l.V=[];
%         l.C=[];
%         l.XYZ=[];
%         %%Move to rotation centre
%         l.V(:,:,1)=lattice.VORTEX(startindex1:endindex1,:,1)-delta_pos(1);
%         l.V(:,:,2)=lattice.VORTEX(startindex1:endindex1,:,2)-delta_pos(2);
%         l.V(:,:,3)=lattice.VORTEX(startindex1:endindex1,:,3)-delta_pos(3);
%
%         l.C(:,1)=lattice.COLLOC(startindex1:endindex1,1)-delta_pos(1);
%         l.C(:,2)=lattice.COLLOC(startindex1:endindex1,2)-delta_pos(2);
%         l.C(:,3)=lattice.COLLOC(startindex1:endindex1,3)-delta_pos(3);
%
%         l.XYZ(:,:,1)=lattice.XYZ(startindex1:endindex1,:,1)-delta_pos(1);
%         l.XYZ(:,:,2)=lattice.XYZ(startindex1:endindex1,:,2)-delta_pos(2);
%         l.XYZ(:,:,3)=lattice.XYZ(startindex1:endindex1,:,3)-delta_pos(3);
%
%
%         lattice.VORTEX(startindex1:endindex1,:,:)=l.V;
%         lattice.COLLOC(startindex1:endindex1,:,:)=l.C;
%         lattice.XYZ(startindex1:endindex1,:,:)=l.XYZ;
%
%         if geo.symetric(wingno)
%             startindex2=vertex_info.startindex2(wingno,i);
%             endindex2=vertex_info.endindex2(wingno,i);
%
%
%             %     pos_now=[lattice.XYZ(vertex_info.K1(wingno,i),1,1) -lattice.XYZ(vertex_info.K1(wingno,i),1,2) lattice.XYZ(vertex_info.K1(wingno,i),1,3)];
%             %     pos_target=[lattice.XYZ(vertex_info.K3(wingno,i-1),2,1) -lattice.XYZ(vertex_info.K3(wingno,i-1),2,2) lattice.XYZ(vertex_info.K3(wingno,i-1),2,3)];
%             delta_pos=delta_pos.*[1 -1 1];
%             %         delta_pos=pos_now-pos_target;
%             %%Move to rotation centre
%             l.V(:,:,1)=lattice.VORTEX(startindex2:endindex2,:,1)-delta_pos(1);
%             l.V(:,:,2)=lattice.VORTEX(startindex2:endindex2,:,2)-delta_pos(2);
%             l.V(:,:,3)=lattice.VORTEX(startindex2:endindex2,:,3)-delta_pos(3);
%
%             l.C(:,1)=lattice.COLLOC(startindex2:endindex2,1)-delta_pos(1);
%             l.C(:,2)=lattice.COLLOC(startindex2:endindex2,2)-delta_pos(2);
%             l.C(:,3)=lattice.COLLOC(startindex2:endindex2,3)-delta_pos(3);
%
%             l.XYZ(:,:,1)=lattice.XYZ(startindex2:endindex2,:,1)-delta_pos(1);
%             l.XYZ(:,:,2)=lattice.XYZ(startindex2:endindex2,:,2)-delta_pos(2);
%             l.XYZ(:,:,3)=lattice.XYZ(startindex2:endindex2,:,3)-delta_pos(3);
%
%
%             lattice.VORTEX(startindex2:endindex2,:,:)=l.V;
%             lattice.COLLOC(startindex2:endindex2,:,:)=l.C;
%             lattice.XYZ(startindex2:endindex2,:,:)=l.XYZ;
%         end
%
%     end
% end



end%function wingrotation
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