function plotvec( startv,endv,color)

vec=squeeze(endv)';
vec1=squeeze(startv)';
vec=vec+vec1;
vecp=[vec1;vec];

hold on

plot3(vecp(:,1),vecp(:,2),vecp(:,3),strjoin({color,'-'}))
plot3(vecp(2,1),vecp(2,2),vecp(2,3),strjoin({color,'*'}))

end



