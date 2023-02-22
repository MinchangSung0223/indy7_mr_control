ptCloud = pcread("monkey.ply")
pcshow(ptCloud);
N = ptCloud.Normal
pts = ptCloud.Location
Nx = N(:,1);
Ny = N(:,2);
Nz = N(:,3);
X = pts(:,1);
Y = pts(:,2);
Z = pts(:,3);
save("monkey.mat","X","Y","Z","Nx","Ny","Nz")