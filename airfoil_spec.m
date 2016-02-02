function [x_cg,z_cg,perimeter,area,Ixx,Iyy,Izz,Ixz] = airfoil_spec(NAME)


[x,z]=airfoil_read(NAME);


% airfoil data nead to be sorted in a circle!!

[ GEOM, INER, CPMO ] = polygeom( x, z ); 

%   GEOM = [ area   X_cen  Y_cen  perimeter ]
%   INER = [ Ixx    Iyy    Ixy    Iuu    Ivv    Iuv ]
%   CPMO = [ I1     ang1   I2     ang2   J ]

x_cg=GEOM(2);
z_cg=GEOM(3);

area=GEOM(1);
perimeter=GEOM(4);

Ixx=INER(4);
Izz=INER(5);
Iyy=CPMO(5);
Ixz=INER(6);



end