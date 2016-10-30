function S2=sortstruct(S)

values=struct2cell(S);
[sortednames,index]=sort(fieldnames(S));

S2=struct(sortednames{1},values(index(1)));

for i=2:length(values)
     S2=setfield(S2,sortednames{i},values{index(i)});
end

end