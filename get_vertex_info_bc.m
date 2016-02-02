function [ vertex_info ] = get_vertex_info(geo,lattice)
k1=1;
nwing=geo.nwing;
startindex1=ones(nwing,max(geo.nelem))*(nan);
endindex1=startindex1;
startindex2=startindex1;
endindex2=startindex1;
mat=lattice.XYZ;

X1=ones(nwing,max(geo.nelem),3)*nan;
X2=ones(nwing,max(geo.nelem),3)*nan;
X3=ones(nwing,max(geo.nelem),3)*nan;
X4=ones(nwing,max(geo.nelem),3)*nan;
% X_LF=X_RF;

for i=1:nwing
    npart=geo.nelem(i);
    for j=1:npart
        
        k2=k1+geo.nx(i,j)-1;
        k2_f=k2+geo.fnx(i,j);
        k3=k1+(geo.nx(i,j)+geo.fnx(i,j))*(geo.ny(i,j)-1);
        k4=k3+geo.nx(i,j)-1;
        k4_f=k4+geo.fnx(i,j);
        
%         hold on
%                 plot3(mat(k1,1,1),mat(k1,1,2),mat(k1,1,3),'r*')
%                 plot3(mat(k2,4,1),mat(k2,4,2),mat(k2,4,3),'r*')
%                 plot3(mat(k3,2,1),mat(k3,2,2),mat(k3,2,3),'r*')
%                 plot3(mat(k4,3,1),mat(k4,3,2),mat(k4,3,3),'r*')
        %
        %         plot3(mat(k2_f,4,1),mat(k2_f,4,2),mat(k2_f,4,3),'b*')
        %         plot3(mat(k4_f,3,1),mat(k4_f,3,2),mat(k4_f,3,3),'b*')
        
        %         X1(l,:)=[mat(k1,1,1) mat(k1,1,2) mat(k1,1,3)];
        %         X2(l,:)=[mat(k2,4,1) mat(k2,4,2) mat(k2,4,3)];
        %         X2_f(l,:)=[mat(k2_f,4,1) mat(k2_f,4,2) mat(k2_f,4,3)];
        %         X3(l,:)=[mat(k3,2,1) mat(k3,2,2) mat(k3,2,3)];
        %         X4(l,:)=[mat(k4,3,1) mat(k4,3,2) mat(k4,3,3)];
        %         X4_f(l,:)=[mat(k4_f,4,1) mat(k4_f,4,2) mat(k4_f,4,3)];
        
        X1(i,j,:)=[mat(k1,1,1) mat(k1,1,2) mat(k1,1,3)];
        X2(i,j,:)=[mat(k2_f,4,1) mat(k2_f,4,2) mat(k2_f,4,3)];
        X3(i,j,:)=[mat(k3,2,1) mat(k3,2,2) mat(k3,2,3)];
        X4(i,j,:)=[mat(k4_f,4,1) mat(k4_f,4,2) mat(k4_f,4,3)];
        
        startindex1(i,j)=k1;
        
        %                 k3=k1+(geo.nx(i,j)+geo.fnx(i,j))*(geo.ny(i,j)-1);
        %         X3(i,j,:)=[mat(k3,2,1) mat(k3,2,2) mat(k3,2,3)];
        
        k1=k1+(geo.nx(i,j)+geo.fnx(i,j))*geo.ny(i,j)-1;
        endindex1(i,j)=k1;
        
        %         X1(i,j,:)=[mat(startindex1(i,j),1,1) mat(startindex1(i,j),1,2) mat(startindex1(i,j),1,3)];
        
        if geo.symetric(i)==1
            k1=k1+1;
            startindex2(i,j)=k1;
            k1=k1+(geo.nx(i,j)+geo.fnx(i,j))*geo.ny(i,j)-1;
            endindex2(i,j)=k1;
            %         X_LF(i,j,:)=[mat(startindex2(i,j),1,1) mat(startindex2(i,j),1,2) mat(startindex2(i,j),1,3)];
        end
        k1=k1+1;
        
        
        %         l=l+1;
    end
    
end
% startindex1
% endindex1
% startindex2
% endindex2
% X_RF
% X_LF


k1=1;
% l=1;
nwing=geo.nwing;

for i=1:nwing
    npart=geo.nelem(i);
    for j=1:npart
        
        %         k2=k1+geo.nx(i,j)-1;
        %         k2_f=k2+geo.fnx(i,j);
        k3=k1+(geo.nx(i,j)+geo.fnx(i,j))*(geo.ny(i,j)-1);
        %         k4=k3+geo.nx(i,j)-1;
        %         k4_f=k4+geo.fnx(i,j);
        
        %         plot3(mat(k1,1,1),mat(k1,1,2),mat(k1,1,3),'r*')
        %         plot3(mat(k2,4,1),mat(k2,4,2),mat(k2,4,3),'r*')
        %         plot3(mat(k3,2,1),mat(k3,2,2),mat(k3,2,3),'r*')
        %         plot3(mat(k4,3,1),mat(k4,3,2),mat(k4,3,3),'r*')
        %
        %         plot3(mat(k2_f,4,1),mat(k2_f,4,2),mat(k2_f,4,3),'b*')
        %         plot3(mat(k4_f,3,1),mat(k4_f,3,2),mat(k4_f,3,3),'b*')
        
        %         X1(i,j,:)=[mat(k1,1,1) mat(k1,1,2) mat(k1,1,3)];
        %         X2(i,j,:)=[mat(k2,4,1) mat(k2,4,2) mat(k2,4,3)];
        %         X2_f(i,j,:)=[mat(k2_f,4,1) mat(k2_f,4,2) mat(k2_f,4,3)];
        %         X3(i,j,:)=[mat(k3,2,1) mat(k3,2,2) mat(k3,2,3)];
        %         X4(i,j,:)=[mat(k4,3,1) mat(k4,3,2) mat(k4,3,3)];
        %         X4_f(i,j,:)=[mat(k4_f,4,1) mat(k4_f,4,2) mat(k4_f,4,3)];
        K1(i,j)=k1;
        K3(i,j)=k3;
        
        k1=k1+(geo.nx(i,j)+geo.fnx(i,j))*geo.ny(i,j);
        
        if geo.symetric(i)==1
            k1=k1+(geo.nx(i,j)+geo.fnx(i,j))*geo.ny(i,j);
        end
        
        %         l=l+1;
    end
    
end



vertex_info.startindex1=startindex1;
vertex_info.endindex1=endindex1;
vertex_info.startindex2=startindex2;
vertex_info.endindex2=endindex2;
vertex_info.X1=X1;
vertex_info.X2=X2;
vertex_info.X3=X3;
vertex_info.X4=X4;

% vertex_info.X_LF=X_LF;
% vertex_info.X3=X3;
vertex_info.K1=K1;
vertex_info.K3=K3;
end

