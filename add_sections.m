function [ geo,lattice,struc ] = add_sections( geo,lattice,struc,wingno,secno)


colnum=max(geo.nelem);

if geo.nelem(wingno)==colnum
    vec=zeros(nwing,1);
    vec(wingno)=geo.b(wingno,secno)/2;
    geo.b(wingno,secno)=geo.b(wingno,secno)/2;
    geo.b=geo.b(:,[1:secno vec secno+1:colnum);
end
geo.nelem(wingno)=geo.nelem(wingno)+1;


end


