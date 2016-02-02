function plot_plane(body,geo,struc)
N=100;
mat=struc.XYZ;
% geo=struc.geo;
% [n,~]=size(mat);
% facecolor=[0.192,0.192,0.192];
% facecolor=[0.224,0.224,0.224];

facecolor=[0.176,0.196,0.222];
% facecolor='k';
% facecolor='w';
edgecolor='none';

geo=struc.geo;
% figure
hold on


% plot3(mat(:,1,1),mat(:,1,2),mat(:,1,3),'k.')
% plot3(mat(:,2,1),mat(:,2,2),mat(:,2,3),'k.')
% plot3(mat(:,3,1),mat(:,3,2),mat(:,3,3),'k.')
% plot3(mat(:,4,1),mat(:,4,2),mat(:,4,3),'k.')

k1=1;
l=1;
nwing=geo.nwing;

[x,y,z] = sphere(5);
scale=0.1;
if struc.pointmass_exist
    for i=1:length(struc.pointmass)
        if ~isempty(struc.pointmass(i).wingno)
            surf(x*scale+struc.pointmass(i).pos(1),y*scale+struc.pointmass(i).pos(2),z*scale+struc.pointmass(i).pos(3),'FaceColor','r','EdgeColor','none')
            if struc.pointmass(i).sym==1
                surf(x*scale+struc.pointmass(i).pos(1),y*scale-struc.pointmass(i).pos(2),z*scale+struc.pointmass(i).pos(3),'FaceColor','r','EdgeColor','none')
            end
            %        plot3(struc.pointmass(i).pos(1),struc.pointmass(i).pos(2),struc.pointmass(i).pos(3),'r*')
        end
    end
end


for i=1:nwing
    npart=geo.nelem(i);
    for j=1:npart
        
        k2=k1+geo.nx(i,j)-1;
        k2_f=k2+geo.fnx(i,j);
        k3=k1+(geo.nx(i,j)+geo.fnx(i,j))*(geo.ny(i,j)-1);
        k4=k3+geo.nx(i,j)-1;
        k4_f=k4+geo.fnx(i,j);
        
        %         plot3(mat(k1,1,1),mat(k1,1,2),mat(k1,1,3),'r*')
        %         plot3(mat(k2,4,1),mat(k2,4,2),mat(k2,4,3),'r*')
        %         plot3(mat(k3,2,1),mat(k3,2,2),mat(k3,2,3),'r*')
        %         plot3(mat(k4,3,1),mat(k4,3,2),mat(k4,3,3),'r*')
        %
        %         plot3(mat(k2_f,4,1),mat(k2_f,4,2),mat(k2_f,4,3),'b*')
        %         plot3(mat(k4_f,3,1),mat(k4_f,3,2),mat(k4_f,3,3),'b*')
        
        X1(l,:)=[mat(k1,1,1) mat(k1,1,2) mat(k1,1,3)];
        X2(l,:)=[mat(k2,4,1) mat(k2,4,2) mat(k2,4,3)];
        X2_f(l,:)=[mat(k2_f,4,1) mat(k2_f,4,2) mat(k2_f,4,3)];
        X3(l,:)=[mat(k3,2,1) mat(k3,2,2) mat(k3,2,3)];
        X4(l,:)=[mat(k4,3,1) mat(k4,3,2) mat(k4,3,3)];
        X4_f(l,:)=[mat(k4_f,4,1) mat(k4_f,4,2) mat(k4_f,4,3)];
        
        
        k1=k1+(geo.nx(i,j)+geo.fnx(i,j))*geo.ny(i,j);
        
        if geo.symetric(i)==1
            k1=k1+(geo.nx(i,j)+geo.fnx(i,j))*geo.ny(i,j);
        end
        
        l=l+1;
    end
    
end

c(:,1)=geo.c';
for i=1:nwing
    npart=geo.nelem(i);
    for j=2:npart+1
        c(i,j)=c(i,j-1)*geo.T(i,j-1);
    end
end

l=0;
for i=1:nwing
    npart=geo.nelem(i);
    for j=1:npart
        l=l+1;
        tr_vec=X3(l,:)-X1(l,:);
        tr_vec=tr_vec/norm(tr_vec);
        af_vec_r=X2(l,:)-X1(l,:);
        af_vec_t=X4(l,:)-X3(l,:);
        
        c_r=c(i,j);
        c_t=c(i,j+1);
        
        af_vec_r=af_vec_r/norm(af_vec_r);
        af_vec_t=af_vec_t/norm(af_vec_t);
        normal_r=cross(af_vec_r,tr_vec);
        normal_t=cross(af_vec_t,tr_vec);
        normal_r=normal_r/norm(normal_r);
        normal_t=normal_t/norm(normal_t);
        
        Tr=[af_vec_r' tr_vec' normal_r'];
        Tt=[af_vec_t' tr_vec' normal_t'];
        
        if geo.fsym(i,j)==1
%             geo.fsym(i,j)
            [xyz_r,xyz_flap_r]=place_foil(Tr,X1(l,:),geo.foil(i,j,1),c_r,geo.flapped(i,j),geo.fc(i,j),geo.flap_vector(i,j),N);
            [xyz_t,xyz_flap_t]=place_foil(Tt,X3(l,:),geo.foil(i,j,2),c_t,geo.flapped(i,j),geo.fc(i,j),geo.flap_vector(i,j),N);
            surface_through_foil(xyz_r,xyz_t,geo.symetric(i),facecolor,edgecolor)
            if geo.flapped(i,j)
                surface_through_foil(xyz_flap_r,xyz_flap_t,geo.symetric(i),facecolor,edgecolor)
            end
        else
            [xyz_r,xyz_flap_r1]=place_foil(Tr,X1(l,:),geo.foil(i,j,1),c_r,geo.flapped(i,j),geo.fc(i,j),geo.flap_vector(i,j),N);
            [~,xyz_flap_r2]=place_foil(Tr,X1(l,:),geo.foil(i,j,1),c_r,geo.flapped(i,j),geo.fc(i,j),-geo.flap_vector(i,j),N);
            [xyz_t,xyz_flap_t1]=place_foil(Tt,X3(l,:),geo.foil(i,j,2),c_t,geo.flapped(i,j),geo.fc(i,j),geo.flap_vector(i,j),N);
            [~,xyz_flap_t2]=place_foil(Tt,X3(l,:),geo.foil(i,j,2),c_t,geo.flapped(i,j),geo.fc(i,j),-geo.flap_vector(i,j),N);
            
            surface_through_foil(xyz_r,xyz_t,geo.symetric(i),facecolor,edgecolor)
            if geo.flapped(i,j)
                surface_through_foil(xyz_flap_r1,xyz_flap_t1,0,facecolor,edgecolor)
                surface_through_foil(xyz_flap_r2,xyz_flap_t2,-1,facecolor,edgecolor)
            end
            
        end
        
    end
end

L=body.length(1);
plot_body(L,facecolor,edgecolor)

xlabel('x')
ylabel('y')
zlabel('z')
daspect([1,1,1]);
view(3);
axis tight equal;
material metal;
camlight;
% lightning flat
end


function [xyz,xyz_flap]=place_foil(T,x,foil,c,isflapped,fc,alpha,N)
[xz,xz_flap]=airfoil_read(foil,isflapped,fc,alpha,N);
[n,~]=size(xz);
xyz0=1*[xz(:,1) zeros(n,1) xz(:,2)];
xyz=[];
xyz_flap=[];
for ii=1:n
    X=x'+T*c*xyz0(ii,:)';
    xyz(ii,:)=X';
end
if isflapped
    [n1,~]=size(xz_flap);
    xyz_f=[xz_flap(:,1) zeros(n1,1) xz_flap(:,2)];
    
    for ii=1:n1
        X=x'+T*c*xyz_f(ii,:)';
        xyz_flap(ii,:)=X';
    end
    
end

end

function surface_through_foil(xyz_r,xyz_t,rls,facecolor,edgecolor)

%%% rls: 1=right -1=left 0=symetric
interiorPoints=[xyz_r;xyz_t];
interiorPoints = unique(interiorPoints,'rows');
DT = delaunayTriangulation(interiorPoints);
hullFacets = convexHull(DT);
if rls==-1
    DT.Points(:,2)=-DT.Points(:,2);
    plot3(xyz_t(:,1),-xyz_t(:,2),xyz_t(:,3),'r')
    plot3(xyz_r(:,1),-xyz_r(:,2),xyz_r(:,3),'r')
else
    plot3(xyz_t(:,1),xyz_t(:,2),xyz_t(:,3),'r')
    plot3(xyz_r(:,1),xyz_r(:,2),xyz_r(:,3),'r')
end

trisurf(hullFacets,DT.Points(:,1),DT.Points(:,2),DT.Points(:,3),'FaceColor',facecolor,'EdgeColor',edgecolor)

if rls==1
    plot3(xyz_t(:,1),-xyz_t(:,2),xyz_t(:,3),'r')
    plot3(xyz_r(:,1),-xyz_r(:,2),xyz_r(:,3),'r')
    
    trisurf(hullFacets,DT.Points(:,1),-DT.Points(:,2),DT.Points(:,3),'FaceColor',facecolor,'EdgeColor',edgecolor)
end

end




