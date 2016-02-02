function [mat]=vector2matrix(vec,m,n)

mat=zeros(m,n);
index=0;
for i=1:m
    for j=1:n
        index=index+1;   
        mat(i,j)=vec(index);

    end
end

end