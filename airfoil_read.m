function [xz,xz_flap]=airfoil_read(foil,isflapped,cf,alpha,N)
% foil must be cell type

if   strcmp('0',char(foil))
    TYPE=3; %flat plate
elseif isempty(str2num((cell2mat(foil))))==0
    TYPE=1;       %Naca xxxx profile, see case 1
elseif isempty(str2num((cell2mat(foil))))
    TYPE=2;       %Airfoil from file, see case 2
    
end

switch TYPE
    case 1
        iaf.designation=char(foil);
        iaf.n=N;
        iaf.HalfCosineSpacing=1;
        iaf.is_finiteTE=0;
        
        af = naca4gen(iaf);
        x=af.x;
        z=af.z;
        
%%%%  sorting data as it is in airfoil files. maybe not neccessary!
        x2=zeros(2*N+2,1);
        z2=zeros(2*N+2,1);
        for i=1:N+1
            x2(i)=x(N-i+2);
            z2(i)=z(N-i+2);
        end
        for i=N+2:2*N+2
            x2(i)=x(i-1);
            z2(i)=z(i-1);
        end
        x=x2;
        z=z2;
        
    case 2
        
        settings=config('startup');
        cd(settings.afdir)
        A=load(char(foil));
        A=A(2:end,:);
        cd(settings.hdir)
        
        x=A(:,1);
        z=A(:,2);
    case 3
        x=[linspace(0,1,N) ones(1,5) linspace(1,0,N) zeros(1,5)];
        z=[-0.01*ones(1,N) linspace(-0.01,0.01,5) 0.01*ones(1,N) linspace(0.01,-0.01,5)];
        
        
end



if isflapped
    n=length(x);
    j1=0;
    j2=0;
    x_unflapped=[];
    z_unflapped=[];
    x_flapped=[];
    z_flapped=[];
    c1=1-cf;
    for i=1:n
        if x(i)<=c1
            j1=j1+1;
            x_unflapped(j1)=x(i);
            z_unflapped(j1)=z(i);
        else
            j2=j2+1;
            x_flapped(j2)=x(i);
            z_flapped(j2)=z(i);
        end
        
    end
    
    ca=cos(alpha);
    sa=sin(alpha);
    T=[ca sa;-sa ca]; %flap down : positive
    n=length(x_flapped);
    ax=min(x_flapped);
    ss=sort(z_flapped);
    az=mean(ss(1:2));
    x_flapped=x_flapped-ax;
    z_flapped=z_flapped-az;
    
    %         ax=max(x_unflapped);
    %     ss=sort(z_unflapped,'descend');
    %     az=mean(ss(1:2));
    %     x_flapped=x_flapped-ax;
    %     z_flapped=z_flapped-az;
    
    for i=1:n
        vec=T*[x_flapped(i);z_flapped(i)];
        x_flapped(i)=vec(1);
        z_flapped(i)=vec(2);
    end
    x_flapped=x_flapped+ax;
    z_flapped=z_flapped+az;
    
    xz=[x_unflapped' z_unflapped'];
    xz_flap=[x_flapped' z_flapped'];
else
    xz=[x z];
    xz_flap=[];
end


end

