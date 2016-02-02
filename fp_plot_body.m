function plot_body(l,facecolor,edgecolor)
% l=30;
xsec=100;
thetasec=30;
method1='PCHIP';
% method1='linear';
method2='PCHIP';
% method2='linear';

xz_up_data=[90.701 186.488
    102.568 173.773
    113.588 166.992
    123.760 162.753
    139.018 161.906
    160.210 164.449
    179.707 166.992
    203.441 167.839
    247.520 167.839
    300.076 167.839
    321.311 166.297
    343.877 169.731
    359.574 173.655
    363.008 177.580];

xz_down_data=[90.752 185.919
    97.129 194.258
    103.016 198.183
    120.185 198.673
    129.996 195.239
    145.694 195.239
    172.184 191.806
    195.730 189.353
    214.371 187.391
    244.295 186.900
    290.406 187.391
    320.821 186.900
    343.877 184.938
    359.574 181.995
    363.499 177.580];

yx_data=[424.817 220.748
    436.100 207.503
    436.100 188.862
    434.629 172.184
    433.157 158.939
    434.138 131.468
    432.666 90.261
    432.666 58.376
    432.176 52.489
    429.723 36.301
    426.780 25.999
    424.327 22.565];


xz_up_data(:,2)=-xz_up_data(:,2);
xz_down_data(:,2)=-xz_down_data(:,2);
xy_data(:,1)=-yx_data(:,2);
xy_data(:,2)=yx_data(:,1);

xz_up_data=sortrows(xz_up_data,1);
xz_down_data=sortrows(xz_down_data,1);
xy_data=sortrows(xy_data,1);

x_right=xy_data(:,1)';
y_right=xy_data(:,2)';
x_up=xz_up_data(:,1)';
z_up=xz_up_data(:,2)';
x_down=xz_down_data(:,1)';
z_down=xz_down_data(:,2)';

scale=l/(x_up(end)-x_up(1));
x_up=(x_up-x_up(1))*scale;
z_up=(z_up-z_up(1))*scale;
scale=l/(x_down(end)-x_down(1));
x_down=(x_down-x_down(1))*scale;
z_down=(z_down-z_down(1))*scale;
scale=l/(x_right(end)-x_right(1));
x_right=(x_right-x_right(1))*scale;
y_right=(y_right-y_right(1))*scale;



x_tot=unique(sort(([x_up x_down x_right])));
z_up_tot=interp1(x_up,z_up,x_tot,method1,'extrap');
z_down_tot=interp1(x_down,z_down,x_tot,method1,'extrap');
y_right_tot=interp1(x_right,y_right,x_tot,'linear','extrap');

xc=x_tot;
for i=1:length(x_tot)
    a(i)=y_right_tot(i);
    b(i)=(z_up_tot(i)-z_down_tot(i))/2;
    zc(i)=z_down_tot(i)+b(i);
end


xc2=linspace(min(xc),max(xc),xsec);
zc2=interp1(xc,zc,xc2,method2);
a2=interp1(xc,a,xc2,method2);
b2=interp1(xc,b,xc2,method2);



x=[];
y=[];
z=[];
theta_ar=linspace(0,2*pi,thetasec);
theta_ar=theta_ar(1:end-1);

j=0;
for i=2:length(xc2)-1
    for theta=theta_ar
        j=j+1;
        r=a2(i)*b2(i)/sqrt((b2(i)*cos(theta))^2+(a2(i)*sin(theta))^2);
        x(j)=xc2(i);
        y(j)=r*cos(theta);
        z(j)=zc2(i)+r*sin(theta);
        
    end
end
i=i+1;
j=j+1;
x(j)=xc2(i);
y(j)=0;
z(j)=zc2(i);

x=[x 0];
y=[y 0];
z=[z 0];



n_c=length(theta_ar);
n_x=(length(x)-2)/n_c;

j=0;
haul=[];
for i=0:n_x-2
    for j=1:n_c-1
        haul=[haul; [i*n_c+j i*n_c+j+n_c i*n_c+j+n_c+1]];
        haul=[haul; [i*n_c+j i*n_c+j+1 i*n_c+j+n_c+1]];
    end
    haul=[haul; [i*n_c+j+1 i*n_c+j+n_c+1 i*n_c+1+n_c]];
    haul=[haul; [i*n_c+j+1 i*n_c+1 i*n_c+1+n_c]];
    
end

last=length(x);
for j=1:n_c-1
    haul=[haul; [last j j+1]];
    
end
haul=[haul; [last n_c 1]];

last=length(x)-1;
N=length(x)-2;
for j=N-n_c+1:N-1
    haul=[haul; [last j j+1]];
    
end
haul=[haul; [last N N-n_c+1]];

% interiorPoints=[x' y' z'];
% DT = delaunayTriangulation(interiorPoints);
% hullFacets = convexHull(DT);
trisurf(haul,x,y,z,'facecolor',facecolor,'edgecolor',edgecolor,'FaceAlpha',0.3)
% trisurf(DT.ConnectivityList,DT.Points(:,1),DT.Points(:,2),DT.Points(:,3))
% % xlabel('x')
% % ylabel('y')
% % zlabel('z')
% % daspect([1,1,1])
% % view(3);
% % axis tight equal
% % material metal
% % camlight
% lightning flat
end