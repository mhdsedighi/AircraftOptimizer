function [struc]=massinertia(struc)
N_airfoil=5;
mat=struc.XYZ;
geo=struc.geo;


if struc.pointmass_exist
    n=length(struc.pointmass);
else
    n=0; 
end
struc.pointmass(n+1).mass=struc.m_fuel;
struc.pointmass(n+1).pos=struc.pos_fuel;


if struc.pointmass_exist
    cg_points_asym=[0;0;0];
    cg_points=[0;0;0];
    m_sum_points_asym=0;
    m_sum_points=0;
    I_points=zeros(3,3);
    
    for i=1:length(struc.pointmass)
        m_this=struc.pointmass(i).mass;
        x=struc.pointmass(i).pos(1);
        y=struc.pointmass(i).pos(2);
        z=struc.pointmass(i).pos(3);
        
        Ixx=struc.pointmass(i).mass*x^2;
        Iyy=struc.pointmass(i).mass*y^2;
        Izz=struc.pointmass(i).mass*z^2;
        Ixy=struc.pointmass(i).mass*x*y;
        Ixz=struc.pointmass(i).mass*x*z;
        Iyz=struc.pointmass(i).mass*y*z;
        
        I_points_this=[Ixx -Ixy -Ixz;-Ixy Iyy -Iyz;-Ixz -Iyz Izz];
        cg_points_asym=cg_points_asym+m_this*struc.pointmass(i).pos(:);
        m_sum_points_asym=m_sum_points_asym+m_this;
        if struc.pointmass(i).sym==1
            m_sum_points=m_sum_points+2*m_this;
            cg_points=cg_points+2*m_this*struc.pointmass(i).pos(:);
            I_points=I_points+I_points_this.*[2 0   2
                0  2  0
                2  0  2];
        else
            m_sum_points=m_sum_points+m_this;
            cg_points=cg_points+m_this*struc.pointmass(i).pos(:);
            I_points=I_points+I_points_this;
        end
        
        
    end
    struc.cg_points_asym=cg_points_asym/m_sum_points_asym;
    struc.cg_points=cg_points/m_sum_points;
    struc.m_sum_points=m_sum_points;
    struc.inertia_points=I_points;
    
else
    struc.cg_points_asym=[0;0;0];
    struc.cg_points=[0;0;0];
    struc.m_sum_points=0;
    struc.inertia_points=zeros(3,3);
end


k1=1;
l=1;
nwing=geo.nwing;


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
        %         %
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
        
        
        
        [xyz_r,~]=place_foil(Tr,X1(l,:),geo.foil(i,j,1),c_r,0,0,0,N_airfoil);
        [xyz_t,~]=place_foil(Tt,X3(l,:),geo.foil(i,j,2),c_t,0,0,0,N_airfoil);
        
        [v,centroid_asym,centroid,inertia]=surface_through_foil(xyz_r,xyz_t,geo.symetric(i));
        
        struc.v(i,j)=v;
        struc.mass(i,j)=v*struc.rho(i,j);
        struc.centroid_asym(i,j,:)=centroid_asym;
        struc.centroid(i,j,:)=centroid;
        struc.inertia(i,j,:,:)=inertia*struc.rho(i,j);
        
        
    end
end

struc.massT=sum(struc.mass);
inertiaT=zeros(3);
massT=0;
xx=[0;0;0];
for i=1:nwing
    npart=geo.nelem(i);
    for j=1:npart
        massT=massT+struc.mass(i,j);
        xx=xx+squeeze(struc.centroid(i,j,:))*struc.mass(i,j);
        inertiaT=inertiaT+squeeze(struc.inertia(i,j,:,:));
    end
end
struc.mass_sections=massT;
struc.cg_sections=xx./massT;
struc.inertia_sections=inertiaT;




struc.mass_all=sum(sum(struc.mass_sections))+struc.m_sum_points+struc.m_body+struc.m_fuel;
struc.cg_all=(struc.cg_sections*struc.mass_sections+struc.cg_points*struc.m_sum_points+struc.cg_body*(struc.m_body+struc.m_fuel))/struc.mass_all;
struc.inertia_all=struc.inertia_sections+struc.inertia_points+struc.inertia_body;

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

function [v,centroid_asym,centroid,inertia]=surface_through_foil(xyz_r,xyz_t,issym)

interiorPoints=[xyz_r;xyz_t];
interiorPoints = unique(interiorPoints,'rows');
DT = delaunayTriangulation(interiorPoints);

% N_mash=length(DT.ConnectivityList);

hullFacets = convexHull(DT);

TR2.vertices=DT.Points;
TR2.faces=hullFacets;
[RBP]=RigidBodyParams(TR2);

centroid_asym=RBP.centroid;
inertia=RBP.inertia_tensor;
v=RBP.volume;

if issym==1
    inertia=inertia.*[2 0   2
        0  2  0
        2  0  2];
    v=2*v;
    centroid=centroid_asym.*[1 0 1];
else
    centroid=centroid_asym;
end





end




