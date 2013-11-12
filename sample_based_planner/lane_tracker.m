function lane_tracker()
close all;

c = rl_init('lane_tracker');
pub = rl_publish('costmap');


KinectHandles = mxNiCreateContext();

R = dlmread('rotation');
T = dlmread('translation');

a = [0; 0];
b = [-500; 500];
X = [1000, 2700];
Y1 = a(1)*X + b(1);
Y2 = a(2)*X + b(2);

global gaussianKern;
global units;
global granularity;
global inflation;

units = 1000;
granularity = 0.05;
inflation = 3;

kern_size = 13;
gaussianKern = makeGaussian(kern_size);


% [I, P] = getImageData(KinectHandles);
% subplot(1, 2, 1), h1 = imagesc(I); axis image;
% subplot(1, 2, 2), h2 = imagesc(I); axis image;
% colormap gray;

while 1
    
    rl_spin(30);

    [I, P] = getImageData(KinectHandles);
    I = getBWImage(I);
    p_ = transform(R, T, I, P);

    [X_, Y1] = biasLane(X, Y1);
    [Y1, lane1] = pullLanes(p_, X_, Y1);
    [Y2, lane2] = pullLanes(p_, X_, Y1+1000);
    obstacles = [lane1; lane2];
    
    costmap = getOccupancyGrid(obstacles);

    msg = Message('costmap', costmap);
    pub.publish(msg);


    % set(h1,'CDATA', I);
    % colormap gray;
    % costmap = flipdim(costmap, 1);
    % set(h2, 'CDATA', costmap); axis image;
    % colormap default;
    % drawnow;
end

end

function costmap = getOccupancyGrid(obstacles)
global gaussianKern;
global units;
global granularity;
global inflation;

tiles = 1/granularity;
scale = units * granularity;
xres = 3*tiles;
yres = 3*tiles;
costmap = zeros(xres, yres);

obstacles = obstacles/scale;
obstacles = round(obstacles);
obstacles(:, 2) = obstacles(:,2) + ceil(yres/2);

% cropping
mask = (obstacles(:,2) > 0 & obstacles(:,2) < yres);
obstacles(:, 1) = obstacles(:,1).*mask;
obstacles(:, 2) = obstacles(:,2).*mask;
obstacles = unique(obstacles, 'rows');
obstacles = setdiff(obstacles, [0 0], 'rows');

cm_idcs = sub2ind([xres yres], obstacles(:,2), obstacles(:,1));
costmap(cm_idcs) = 1;

costmap = bwmorph(costmap, 'dilate', inflation);
costmap = conv2(double(costmap), gaussianKern, 'same');

% keyboard;

end

function [X_, Y_] = biasLane(X, Y)
    theta = pi/18;
    R = [cos(theta) sin(theta); -sin(theta) cos(theta)];
    P_ = R*[X' Y'];
    P_ = P_';
    X_ = P_(1,:);
    Y_ = P_(2,:) + 10;
end

function [Y, inliers] = pullLanes(data, X, Y)

    xpnts = [X(1)-150 X(1)-150 X(2)+150 X(2)+150];
    ypnts = [Y(1)+150 Y(1)-150 Y(2)-150 Y(2)+150];

    inliers_mask = inpolygon(data(:,1), data(:,2), xpnts, ypnts);
    inliers = data(inliers_mask,:);
    
    [a, b, inliers] = ransac(inliers);
    
    Y = a*X + b;
end


function [a, b, c_set] = ransac(data)

n = 2;
k = 40;
t = 50;

c_set = [];
maxsize = 0;

for i=1:k
    idc = randperm(size(data,1), n);
    p1 = data(idc(1),:);
    p2 = data(idc(2),:);
    a = (p2(2)-p1(2))/(p2(1)-p1(1));
    b = p2(2) - a*p2(1);
    
    y = a*data(:,1) + b;
    e = abs(data(:,2) - y);

    mask = (e < t);
    e = e.*mask;
    len = size(unique(e), 1) - 1;

    if len > maxsize 
        X = data(:,1).*mask; 
        Y = data(:,2).*mask; 
        c_set = [X Y];
        maxsize = len;
    end
end

c_set = unique(c_set, 'rows');
c_set = setdiff(c_set, [0 0], 'rows');
[a, b] = refit(c_set);

end

function [a, b] = refit(data)
x = data(:,1);
y = data(:,2);

C = [x ones(size(x))];
p = C\y;

a = p(1);
b = p(2);
end

function p = transform(R, T, I, P)
X = P(:,:,1).*I;
Y = P(:,:,2).*I;
Z = P(:,:,3).*I;

d_mask = (Z < 2500);
X = X.*d_mask;
Y = Y.*d_mask;
Z = Z.*d_mask;

X = reshape(X, 1, 240*320);
Y = reshape(Y, 1, 240*320);
Z = reshape(Z, 1, 240*320);

ps = [X; Y; Z];
ps_ = R'*ps; 
ps_ = ps_';

ps_(:,1) = ps_(:,1) + T(1);
ps_(:,2) = ps_(:,2) + T(2);

ps_ = [ps_(:,1) ps_(:,2)];
ps_ = unique(ps_, 'rows');
ps_ = setdiff(ps_, [0 0], 'rows');

p = ps_;

end

function bw = getBWImage(I)
I = rgb2gray(I);
bw = edge(I, 'sobel');
bw(120:end, :) = 0;
bw(1:50, :) = 0;
end

function K = makeGaussian(n)
K = [1 1];

for i=3:n
    K = conv([1 1], K);
end

K = conv2(K, K');
K = K/sum(sum(K));
end

function [I, P] = getImageData(KinectHandles)
I = getRGBImage(KinectHandles);
P = double(getPointCloud(KinectHandles));
end
