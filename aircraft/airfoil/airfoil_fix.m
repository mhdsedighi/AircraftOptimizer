[n,~]=size(M);
M2=[];
for i=1:n
   M2(n-i+1,:)=M(i,:); 
end