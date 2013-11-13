function lane_tracker()
close all;

c = rl_init('lane_tracker');
pub = rl_publish('costmap');

i = 0;
% KinectHandles = mxNiCreateContext();

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
inflation = 2;

kern_size = 11;
gaussianKern = makeGaussian(kern_size);


% [I, P] = getImageData(KinectHandles);
[I, P] = getImageData(i);
subplot(1, 2, 1), h1 = imagesc(I); axis image;
subplot(1, 2, 2), h2 = plot(X, Y1); hold on; plot(X, Y2);
axis([0 3000, -1500 1500]);
colormap gray;

while 1
    
    rl_spin(30);

    % [I, P] = getImageData(KinectHandles);
    [I, P] = getImageData(i);
    I = getBWImage(I);
    p_ = transform(R, T, I, P);

    % [X_, Y1] = biasLane(X, Y1);
    X_ = X;
    [Y1, lane1] = pullLanes(p_, X_, Y1);
    [Y2, lane2] = pullLanes(p_, X_, Y1+1000);
    obstacles = [lane1; lane2];
    
    costmap = getOccupancyGrid(obstacles);

    msg = Message('costmap', costmap);
    pub.publish(msg);


    i = i + 1;
    set(h1,'CDATA', I);
    colormap gray;
    cla(h2);
    set(h2, 'XDATA', X, 'YDATA', Y1); hold on; plot(X, Y2); 
    plot(obstacles(:,1), obstacles(:,2), 'b.'); axis equal;
    axis([0 3000, -1500 1500]); 
    drawnow;
end

end

% creates tan occupancy grid which is passed into the planner
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

% place obstacles in bins
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

% inflation of obstacles
costmap = bwmorph(costmap, 'dilate', inflation);
costmap = conv2(double(costmap), gaussianKern, 'same');
end

% hack to bias the region of interest towards the left
function [X_, Y_] = biasLane(X, Y)
    theta = pi/18;
    R = [cos(theta) sin(theta); -sin(theta) cos(theta)];
    P_ = R*[X' Y'];
    P_ = P_';
    X_ = P_(1,:);
    Y_ = P_(2,:) + 10;
end

% extracts lans from a region of interest
function [Y, inliers] = pullLanes(data, X, Y)

    xpnts = [X(1)-150 X(1)-150 X(2)+150 X(2)+150];
    ypnts = [Y(1)+150 Y(1)-150 Y(2)-150 Y(2)+150];

    inliers_mask = inpolygon(data(:,1), data(:,2), xpnts, ypnts);
    inliers = data(inliers_mask,:);
    
    [a, b, inliers] = ransac(inliers);
    
    Y = a*X + b;
end


% ransac finds the best line without outliers using the ransac algorithm
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

% refit uses least squares to find the best parameters for a line from a set of points
function [a, b] = refit(data)
x = data(:,1);
y = data(:,2);

C = [x ones(size(x))];
p = C\y;

a = p(1);
b = p(2);
end

% transform uses the calibration to transfrom a pointcloud from the sensor 
%   frame to our car frame
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

% basic image processing on our raw image, returns the bitmap of edges
function bw = getBWImage(I)
I = rgb2gray(I);
bw = edge(I, 'sobel');
bw(120:end, :) = 0;
bw(1:50, :) = 0;
end

% creates a gaussian kernel which is used to create an exponential decay fx
% around our inflated obstacle
function K = makeGaussian(n)
K = [1 1];

for i=3:n
    K = conv([1 1], K);
end

K = conv2(K, K');
K = K/sum(sum(K));
end

function [I, P] = getImageData(i)
folder = 'Image-10-17/ImageLapData/';
im_file = strcat('/home/armon/Documents/robot_nascar/data/',folder,'i', num2str(i), '+1i.png');
pc_file = strcat('/home/armon/Documents/robot_nascar/data/',folder,'i', num2str(i), '+1i.mat');
I = imread(im_file);
P = load(pc_file, '-mat');
P = P.P;
end
% function [I, P] = getImageData(KinectHandles)
% I = getRGBImage(KinectHandles);
% P = double(getPointCloud(KinectHandles));
% end
