function [geo,lattice,struc]=airfoil_bend(geo,lattice,struc,wingno,borderno,pivot_portion,alpha)

global xz xz2

N=10; %% alaki!
foil=geo.airfoil(wingno,borderno);
[xz,~]=airfoil_read(foil,0,0,0,N);

[n,~]=size(xz);
% x=xz(:,1);
% z=xz(:,2);


Xc=[pivot_portion 0];

is_in=zeros(1,n);
farness=zeros(1,n);

for i=1:n
    %    vec=xz(i,:)-Xc;
    if xz(i,1)>=pivot_portion
        is_in(i)=1;
        farness(i)=xz(i,1)-pivot_portion;
    end
end
farness=farness./max(farness);

xz2=xz;

for i=1:n
    if is_in(i)==1;
        alpha2=farness(i)*alpha;
        T=[cosd(alpha2) sind(alpha2);-sind(alpha2) cosd(alpha2)];
        vec=xz(i,:)-Xc;
        vec=(T*vec')';
        vec=vec+Xc;
        xz2(i,:)=vec;
    end
end

x2=xz2(:,1);
z2=xz2(:,2);

%%% making airfoil chord 1
c_new2old=max(x2);
x2_af=x2/c_new2old;
xz_af=[x2_af z2];





nsec=geo.nelem(wingno);
switch  borderno
    case 1
%         geo.foil(wingno,1,1)=geo.airfoil(wingno,borderno); why?
        new_name=strjoin({'airfoil' num2str(wingno) num2str(1) num2str(1) '.DAT'});
        airfoil_write(xz_af,new_name)
    case nsec+1
%         geo.foil(wingno,nsec,2)=geo.airfoil(wingno,borderno);
        new_name=strjoin({'airfoil' num2str(wingno) num2str(nsec) num2str(2) '.DAT'});
        airfoil_write(xz_af,new_name)
    otherwise
%         geo.foil(wingno,borderno-1,2)=geo.airfoil(wingno,borderno);
%         geo.foil(wingno,borderno,1)=geo.airfoil(wingno,borderno);
        new_name=strjoin({'airfoil' num2str(wingno) num2str(borderno-1) num2str(2) '.DAT'});
        airfoil_write(xz_af,new_name)
        new_name=strjoin({'airfoil' num2str(wingno) num2str(borderno) num2str(1) '.DAT'});
        airfoil_write(xz_af,new_name)
end

struc.geo.foil=geo.foil;
if borderno==1
    secno=1;
    geo.c(wingno,secno)=geo.c(wingno,secno)*c_new2old;
    geo.T(wingno,secno)=geo.T(wingno,secno)/c_new2old;
else
    secno=borderno-1;
    geo.T(wingno,secno)=geo.T(wingno,secno)*c_new2old;
    if secno<nsec
        geo.T(wingno,secno+1)=geo.T(wingno,secno+1)/c_new2old;
    end
end
struc.geo.c=geo.c;
struc.geo.T=geo.T;





end