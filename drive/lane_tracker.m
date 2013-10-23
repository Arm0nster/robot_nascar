function lane_tracker()

c = rl_init('lane_tracker');
pub = rl_publish('pose');

close all;
% global kbhit;
% kbhit = false;
% figure('KeyPressFcn', @my_kbhit);
figure;

KinectHandles = mxNiCreateContext();

[I, P] = getImageData(KinectHandles);

pic = I;
I = getBWImage(I);

R = dlmread('rotation');
T = dlmread('translation');

p_ = transform(R, T, I, P);
[c_set1, a1, b1] = ransac(p_);
p_2 = setdiff(p_, c_set1, 'rows');
[c_set2, a2, b2] = ransac(p_2);
c_set = [c_set1; c_set2];

subplot(2, 2, 1), h1 = imagesc(pic);axis image;
subplot(2, 2, 2), h2 = imagesc(I); axis image;
colormap gray;
subplot(2, 2, 3), h3 = plot(p_(:,1), p_(:,2), '.');
axis([0 3000, -1500 1500]);
subplot(2, 2, 4), h4 = plot(c_set(:,1), c_set(:,2), '.');
axis([0 3000, -1500 1500]);

while 1
% while ~kbhit
% for k=1:1
    rl_spin(25);

    [I, P] = getImageData(KinectHandles);
    pic = I;

    I = getBWImage(I);

    p_ = transform(R, T, I, P);
    [c_set1, a1, b1] = ransac(p_);
    p_2 = setdiff(p_, c_set1, 'rows');
    [c_set2, a2, b2] = ransac(p_2);
    c_set = [c_set1; c_set2];

    [y, theta] = getPose(c_set1, a1, b1, c_set2, a2, b2);
    y = y/1000;
    
    pose = [y, theta];
    msg = Message('pose', pose);
    pub.publish(msg);

    set(h1,'CDATA', pic);
    set(h2, 'CDATA', I);
    colormap gray;
    set(h3, 'XDATA', p_(:,1), 'YDATA', p_(:,2));
    axis([0 3000, -1500 1500]);
    set(h4, 'XDATA', c_set(:,1), 'YDATA', c_set(:,2));
    axis([0 3000, -1500 1500]);
    drawnow;
end

mxNiDeleteContext(KinectHandles);

end

function [y, theta] = getPose(set1, a1, b1, set2, a2, b2)
y1 = a1*set1(:,1) + b1;
y2 = a2*set2(:,1) + b2;

y1_bar = mean(y1);
y2_bar = mean(y2);

y = (y1_bar + y2_bar)/2;

theta = 0;
if (y1_bar > 0)
    theta = atan(a1);
else
    if(y2_bar > 0)
        theta = atan(a2);
    end
end

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

function [c_set, a, b] = ransac(data)
% TODO:
% save largest potential lines
% pick largest
% find dot product of largest and the next largest st. dp = 0

n = 2;
k = 50;
t = 70;

c_set = [];
maxsize = 0;

samp_space = [];

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
        samp_space = cat(3, samp_space, c_set);
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


function bw = getBWImage(I)

thresh = 8;

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
