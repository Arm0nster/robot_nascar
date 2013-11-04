function lane_tracker()

c = rl_init('lane_tracker');
pub = rl_publish('pose');

close all;
% global kbhit;
% kbhit = false;
% figure('KeyPressFcn', @my_kbhit);
% figure;

KinectHandles = mxNiCreateContext();

[I, P] = getImageData(KinectHandles);

pic = I;
I = getBWImage(I);

R = dlmread('rotation');
T = dlmread('translation');

p_ = transform(R, T, I, P);
a = [0; 0];
b = [-500; 500];

X = [1000, 2700];
Y1 = a(1)*X + b(1);
Y2 = a(2)*X + b(2);


% subplot(2, 2, 1), h1 = imagesc(pic);axis image;
% subplot(2, 2, 2), h2 = imagesc(I); axis image;
% colormap gray;
% subplot(2, 2, 3), h3 = quiver(0, 1000, 0, 500, 'r', 'LineWidth', 2); 
% hold on; plot([0 3000], [500 500]); plot([0 3000], [-500 -500]);
% axis([0 3000, -1500 1500]);
% subplot(2, 2, 4), h4 = plot(X, Y1); hold on; plot(X, Y2);
% axis([0 3000, -1500 1500]);

while 1
% while ~kbhit
% for k=1:1
    rl_spin(20);

    [I, P] = getImageData(KinectHandles);
    pic = I;
    I = getBWImage(I);
    p_ = transform(R, T, I, P);

    [Y1, xv1, yv1] = pullLanes(p_, X, Y1);
    [Y2, xv2, yv2] = pullLanes(p_, X, Y1+1000);

    [y, theta] = getPose(X, Y1, Y2);
    y = y/1000;
    
    pose = [y, theta];
    msg = Message('pose', pose);
    pub.publish(msg);

    car_x = 200;
    car_y = car_x*tan(theta);

    set(h1,'CDATA', pic);
    set(h2, 'CDATA', I);
    colormap gray;
    set(h3, 'XDATA', 1000, 'YDATA', y, 'VDATA', car_y, 'UDATA', car_x); 
    axis([0 3000, -1500 1500]);
    cla(h4);
    set(h4, 'XDATA', X, 'YDATA', Y1); hold on; plot(X, Y2, 'r'); 
    plot(xv1, yv1); plot(xv2, yv2, 'r'); plot(p_(:,1), p_(:,2), '.');
    axis([0 3000, -1500 1500]);
    drawnow;
end

mxNiDeleteContext(KinectHandles);

end

function [y, theta] = getPose(X, Y1, Y2)
i = (X(2)-X(1));
j1 = (Y1(2)-Y1(1));
j2 = (Y2(2)-Y2(1));

r1 = [i j1];
r2 = [i j2];
r = r1 + r2;

basis = [1 0];
r = r/norm(r);

theta = acos(dot(basis, r));

m = r(2)/r(1);

if m > 0 
    theta = -1 * theta;
end

[a1 b1] = refit([X' Y1']);
[a2 b2] = refit([X' Y2']);

r_orth = [-1*r(2) r(1)]; 
m_orth = r_orth(2)/r_orth(1);
b_orth = -1*m_orth*X(1);
intcp = [(b1-b_orth)/(m_orth-a1), ((b1-b_orth)/(m_orth-a1)*m_orth) + b_orth; ...
        (b2-b_orth)/(m_orth-a2), ((b2-b_orth)/(m_orth-a2)*m_orth) + b_orth];

width = dist(intcp);
d_bot = dist([X(1), 0; intcp(1, :)]);
d_top = dist([X(1), 0; intcp(2, :)]);


% y = 500 - 1000*(d_top/width);
y = 500 - d_top*(width/1000);

end

function [Y, xpnts, ypnts] = pullLanes(data, X, Y)

    xpnts = [X(1)-150 X(1)-150 X(2)+150 X(2)+150];
    ypnts = [Y(1)+150 Y(1)-150 Y(2)-150 Y(2)+150];

    inliers_mask = inpolygon(data(:,1), data(:,2), xpnts, ypnts);
    inliers = data(inliers_mask,:);
    
    [a, b] = ransac(inliers);
    
    Y = a*X + b;
end


function [a, b] = ransac(data)
n = 2;
k = 50;
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

function [a, b] = refit(data)
x = data(:,1);
y = data(:,2);

C = [x ones(size(x))];
p = C\y;

a = p(1);
b = p(2);
end

function r = dist(pts)

    r = sum((pts(2, :) - pts(1, :)).^2);
    r = r.^(1/2);

end


function bw = getBWImage(I)
I = rgb2gray(I);
bw = edge(I, 'sobel');
bw(120:end, :) = 0;
bw(1:50, :) = 0;
end

function [I, P] = getImageData(KinectHandles)
I = getRGBImage(KinectHandles);
P = double(getPointCloud(KinectHandles));
end
