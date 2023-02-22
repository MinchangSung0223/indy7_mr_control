clf;

t = 0:0.2:2*pi;
r = 0.1+0.1*cos(t);
[X,Y,Z] = cylinder(r,50);
Z = Z/2
surf2stl('cylinder.stl',X,Y,Z)
surfnorm(X,Y,Z,'FaceAlpha',0.4,'EdgeAlpha',0);hold on;
view(-60 ,45)
daspect([1,1,1])
[Nx,Ny,Nz] = surfnorm(X,Y,Z);

daspect([1,1,1])
f = [10 10 10]'
m = [10 10 10]'

norm_f = norm(f);
u = f/norm_f
% plot3([-u(1)*10, -u(1)*10],[ -u(2)*10, u(2)*10],[-u(3)*10, u(3)*10],'g-')
X = reshape(X,[],1)
Y = reshape(Y,[],1)
Z = reshape(Z,[],1)
Nx = reshape(Nx,[],1)
Ny = reshape(Ny,[],1)
Nz = reshape(Nz,[],1)
save("cylinder.mat","X","Y","Z","Nx","Ny","Nz")
Estimate_m=[]
dist_m = []
for i = 1:1:length(Nx)
    U = [0 -u(3) u(2);
        u(3) 0 -u(1);
        -u(2) u(1) 0;
        Nx(i) Ny(i) Nz(i)];
    d = norm([X(i),Y(i),Z(i)]);
    est_m = -U*[X(i) Y(i) Z(i)]'*norm_f;
    Estimate_m= [Estimate_m;est_m(1:3)'];
    dist_m=[dist_m, norm(est_m(1:3)-m)];
end

[val,ind]=mink(dist_m,21)
cmap = colormap(jet(21))
prev_val = 9999999999
color_count = 1;
for i = 1:1:21
    if prev_val >= dist_m(ind)
        plot3(X(ind(i)),Y(ind(i)),Z(ind(i)),"o","LineWidth",10,"Color",cmap(color_count,:))
        color_count = color_count+1;
        prev_val = dist_m(ind);
    else

        continue;
    end
end
grid on;


