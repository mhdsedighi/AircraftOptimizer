
% clc
close all
N=20;
% [xz,xz_flap]=airfoil_read({'FOIL31.DAT'},0,0,0,N);
% [xz,xz_flap]=airfoil_read({'testaf.DAT'},0,0,0,N);
[xz,xz_flap]=airfoil_read({'0012'},0,0,0,N);

[n,~]=size(xz)

x=xz(:,1)
z=xz(:,2)
pivot_portion=0.6;
alpha=30;

Xc=[pivot_portion 0];



is_in=zeros(1,n);
farness=zeros(1,n);
% for i=1:n
%    vec=xz(i,:)-Xc;
%    theta=atan2d(vec(2),vec(1));
%    if theta>=theta_min && theta<=theta_max
%        is_in(i)=1;
%    end
% end
for i=1:n
   vec=xz(i,:)-Xc;
   if x(i)>=pivot_portion
       is_in(i)=1;
       farness(i)=x(i)-pivot_portion;
   end
end
xz2=xz;
farness=farness./max(farness);
% T=[cosd(alpha) sind(alpha);-sind(alpha) cosd(alpha)];
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
c_new=max(x2);

x2_af=x2/c_new;
z2_af=z2;
xz_af=[x2_af z2_af];

dlmwrite('test.txt',xz_af,' ')

figure
hold on

for i=1:n
   if is_in(i)==1
       plot(x(i),z(i),'k.');
       plot(x2(i),z2(i),'r.');
   else
       plot(x(i),z(i),'k.');
   end
end

plot(x,z);
plot(x2,z2);

plot(Xc(1),Xc(2),'r*');
axis equal
grid
