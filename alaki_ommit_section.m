function mat2=ommit_section(mat1,wingno,partno)

[m,n]=size(mat1);
mat2=mat1;
mat2(wingno,partno)=0;

is_all_zero=1;

for i=1:m
   if mat2(i,partno)~=0
      is_all_zero=0;
      break
   end
end

if is_all_zero
    mat2=mat2(:,[1:partno-1 partno+1:n]);
end





end