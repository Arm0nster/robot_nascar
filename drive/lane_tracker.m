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
p_a = [0; 0];
p_b = [-500; 500];

X = [1000, 2000];
Y1 = p_a(1)*X + p_b(1);
Y2 = p_a(2)*X + p_b(2);


% subplot(2, 2, 1), h1 = imagesc(pic);axis image;
% subplot(2, 2, 2), h2 = imagesc(I); axis image;
% colormap gray;
% subplot(2, 2, 3), h3 = plot(p_(:,1), p_(:,2), '.');
% axis([0 3000, -1500 1500]);
% subplot(2, 2, 4), h4 = plot(X, Y1); hold on; plot(X, Y2);
% axis([0 3000, -1500 1500]);

while 1
% while ~kbhit
% for k=1:1
    rl_spin(25);

    [I, P] = getImageData(KinectHandles);
    pic = I;

    I = getBWImage(I);

    p_ = transform(R, T, I, P);
    [c_set, a, b] = pullLanes(p_);
    [a, b] = filter(a, b, p_a, p_b);
    p_a = a;
    p_b = b;

    X = [1000, 2000];
    Y1 = a(1)*X + b(1);
    Y2 = a(2)*X + b(2);

    [y, theta] = getPose(X, Y1, Y2);
    y = y/1000;
    
    pose = [y, theta];
    msg = Message('pose', pose);
    pub.publish(msg);

    % set(h1,'CDATA', pic);
    % set(h2, 'CDATA', I);
    % colormap gray;
    % set(h3, 'XDATA', p_(:,1), 'YDATA', p_(:,2));
    % axis([0 3000, -1500 1500]);
    % cla(h4);
    % set(h4, 'XDATA', X, 'YDATA', Y1); hold on; plot(X, Y2);
    % axis([0 3000, -1500 1500]);
    % drawnow;
end

mxNiDeleteContext(KinectHandles);

end

function [y, theta] = getPose(X, Y1, Y2)
i = (X(2)-X(1));
j = (Y2(2)-Y2(1));
r = [i j];
b = [1 0];
r_ = r/norm(r);


theta = acos(dot(r_, b));
alph = (pi/2) - theta;

dist = Y2(1)/sin(alph);
y = 500 - dist;

% y = ((mean(Y2) + mean(Y1))/2);
% % y = mean(Y2) - 600; 
% m = (Y2(2)-Y2(1))/(X(2)-X(1));
% theta = -1*atan(m) - pi/25;

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

function [a, b] = filter(a, b, p_a, p_b)
d_a = abs(p_a - a);
d_b = abs(p_b - b);

if (d_a > pi/20) & (d_b > 80)
    a = p_a;
    b = p_b;
else
    a = a;
    b = b;
end

end

function [c_set, a, b] = ransac(data)
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

function [set, a, b] = pullLanes(data)
thresh = pi/20;
k = 4;

set  = [];

[c_set, a_, b_] = ransac(data);
x = [1; 2];
y = a_*x + b_;
r_x = x(2)-x(1);
r_y = y(2)-y(1);
th_ = atan(r_y/r_x);
data = setdiff(data, c_set, 'rows');
set = [set; c_set];

for i=1:k
    [c_set, a, b] = ransac(data);
    x = [1; 2];
    y = a*x + b;
    r_x = x(2)-x(1);
    r_y = y(2)-y(1);
    th = atan(r_y/r_x);
    if abs(th_-th) < thresh && abs(b_- b) > 600
        set = [set; c_set];
        break;
    end
    data = setdiff(data, c_set, 'rows');
end
a = [a_; a];
b = [b_; b];

[b,idc] = sort(b);
a = a(idc);

end

function [a, b] = refit(data)
x = data(:,1);
y = data(:,2);

C = [x ones(size(x))];
p = C\y;

a = p(1);
b = p(2);
end


function bw = getBWImage(I)

thresh = 9;

I(1:50,:,:) = 0;
I(130:end,:,:) = 0;

I = rgb2gray(I);
dx = conv2(double(I), [1 0; 0 -1], 'same');
dy = conv2(double(I), [0 1; -1 0], 'same');
mag = sqrt(dx.^2 + dy.^2);
I = (mag>thresh);
I = 255*I;
bw = bwmorph(I, 'OPEN', 2);

end

function [I, P] = getImageData(KinectHandles)
I = getRGBImage(KinectHandles);
P = double(getPointCloud(KinectHandles));
end
