function [vec]=matrix2vector(mat,m,n)

index=0;
for i=1:m
    for j=1:n
        index=index+1;
        vec(index)=mat(i,j);
    end
end

end