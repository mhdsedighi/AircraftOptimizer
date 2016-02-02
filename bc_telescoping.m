function [lattice]=telescoping(wingno,partno,geo,lattice,struc,course_portion)

vertex_info=lattice.vertex_info;
svertex_info=struc.vertex_info;
sgeo=struc.geo;

wing_parts=sgeo.nelem(wingno);

rail_partno=partno-1;
rail_end=(svertex_info.X4_f(wingno,rail_partno,:)+svertex_info.X3(wingno,rail_partno,:))/2;
rail_start=(svertex_info.X2_f(wingno,rail_partno,:)+svertex_info.X1(wingno,rail_partno,:))/2;
rail_vec=rail_end-rail_start;

% plotvec( rail_start,rail_vec,'r')
%     course_max=norm(rail_vec);

course_vec=course_portion*rail_vec;


for p=partno:wing_parts

startindex1s=svertex_info.startindex1(wingno,p);
endindex1s=svertex_info.endindex1(wingno,p);



%%Move to rotation centre
l.V(:,:,1)=lattice.VORTEX(startindex1s:endindex1s,:,1)-course_vec(1);
l.V(:,:,2)=lattice.VORTEX(startindex1s:endindex1s,:,2)-course_vec(2);
l.V(:,:,3)=lattice.VORTEX(startindex1s:endindex1s,:,3)-course_vec(3);

l.C(:,1)=lattice.COLLOC(startindex1s:endindex1s,1)-course_vec(1);
l.C(:,2)=lattice.COLLOC(startindex1s:endindex1s,2)-course_vec(2);
l.C(:,3)=lattice.COLLOC(startindex1s:endindex1s,3)-course_vec(3);

l.XYZ(:,:,1)=lattice.XYZ(startindex1s:endindex1s,:,1)-course_vec(1);
l.XYZ(:,:,2)=lattice.XYZ(startindex1s:endindex1s,:,2)-course_vec(2);
l.XYZ(:,:,3)=lattice.XYZ(startindex1s:endindex1s,:,3)-course_vec(3);

l.N(:,:,:)=lattice.N(startindex1s:endindex1s,:,:);

lattice.VORTEX(startindex1s:endindex1s,:,:)=l.V;
lattice.COLLOC(startindex1s:endindex1s,:,:)=l.C;
lattice.XYZ(startindex1s:endindex1s,:,:)=l.XYZ;
lattice.N(startindex1s:endindex1s,:,:)=l.N;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if geo.symetric(wingno)
    startindex2s=vertex_info.startindex2(wingno,p);
    endindex2s=vertex_info.endindex2(wingno,p);
    course_vec2=[1;-1;1].*squeeze(course_vec);

    
    %%Move to rotation centre
    l.V(:,:,1)=lattice.VORTEX(startindex2s:endindex2s,:,1)-course_vec2(1);
    l.V(:,:,2)=lattice.VORTEX(startindex2s:endindex2s,:,2)-course_vec2(2);
    l.V(:,:,3)=lattice.VORTEX(startindex2s:endindex2s,:,3)-course_vec2(3);
    
    l.C(:,1)=lattice.COLLOC(startindex2s:endindex2s,1)-course_vec2(1);
    l.C(:,2)=lattice.COLLOC(startindex2s:endindex2s,2)-course_vec2(2);
    l.C(:,3)=lattice.COLLOC(startindex2s:endindex2s,3)-course_vec2(3);
    
    l.XYZ(:,:,1)=lattice.XYZ(startindex2s:endindex2s,:,1)-course_vec2(1);
    l.XYZ(:,:,2)=lattice.XYZ(startindex2s:endindex2s,:,2)-course_vec2(2);
    l.XYZ(:,:,3)=lattice.XYZ(startindex2s:endindex2s,:,3)-course_vec2(3);
    
    l.N(:,:,:)=lattice.N(startindex2s:endindex2s,:,:);
   
    
    
    lattice.VORTEX(startindex2s:endindex2s,:,:)=l.V;
    lattice.COLLOC(startindex2s:endindex2s,:,:)=l.C;
    lattice.XYZ(startindex2s:endindex2s,:,:)=l.XYZ;
    lattice.N(startindex2s:endindex2s,:,:)=l.N;
end


end
%%%%%%%










end%function wingrotation