function [R, t] = calibrate()

close all;

global kbhit;
kbhit = false;

KinectHandles=mxNiCreateContext();

figure('KeyPressFcn', @my_kbhit);
I = getRGBImage(KinectHandles);
D = getPointCloud(KinectHandles);
h=imagesc(I); 

while ~kbhit
    I = getRGBImage(KinectHandles);
    D = getPointCloud(KinectHandles);
    z = D(:,:,3);                                                             
    z = (z ~= 0);                                                             
    r = I(:,:,1).*uint8(z);                                                   
    g = I(:,:,2).*uint8(z);                                                   
    b = I(:,:,3).*uint8(z);                                                   
    I = cat(3,r,g,b);
    set(h,'CDATA',I);
    drawnow; 
end

[y_idcs, x_idcs] = ginput;
x_idcs = round(x_idcs);
y_idcs = round(y_idcs);
ps = [];
for i=1:size(x_idcs, 1)
    ps = [ps; D(x_idcs(i), y_idcs(i), :)];
end
dlmwrite('sensor_points', ps, '\t');
figure;
scatter3(ps(:,:,1), ps(:,:,2), ps(:,:,3), 9, ps(:,:,3));
axis([-2000 2000 -2000 2000 -2000 2000]);

X = reshape(D(:,:,1), (240*320),1);
Y = reshape(D(:,:,2), (240*320),1);
Z = reshape(D(:,:,3), (240*320),1);
figure;
scatter3(X, Y, Z, 9, Z);
axis([-2000 2000 -2000 2000 -2000 2000]);

p = dlmread('clicked_points');
pv = world_points(p);
dlmwrite('vehicle_points', pv, '\t');

get_transformation(ps, pv);

mxNiDeleteContext(KinectHandles);
end

