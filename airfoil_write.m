function airfoil_write(xz,af_name)
[n,~]=size(xz);
n=n/2;

XZ=[n n];
XZ=[XZ;xz];

settings=config('startup');
cd(settings.afdir)
dlmwrite(af_name,XZ,'delimiter',' ');
cd(settings.hdir)

end

