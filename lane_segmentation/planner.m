function planner()

c = rl_init('planner');
sub = rl_subscribe('costmap');
pub = rl_publish('control');

% figure;
% h = imagesc(zeros(1));
while 1
    rl_spin(10);
    msg = sub.getLatestMessage();
    if isempty(msg)
        continue;
    end
    costmap = msg.data;
    
    control = findBestTrajectory(costmap);
    disp(control);

    % set(h, 'CDATA', costmap); axis image;
end

end

function [opt_w] = findBestTrajectory(costmap)

w_gain = 35;
obstacle_gain = 100;

costmap_res = 0.05;
x_res = size(costmap, 2); 
y_res = size(costmap, 1); 

sim_granularity = 21;
v = 3.5;

w_max = 1;
w_granularity = 27; 
w_space = linspace(-w_max, w_max, w_granularity);

R = v./w_space';
th = linspace(0, pi/6, sim_granularity);
s = sin(th);

r = R*s;

X = r.*cos(repmat(th, [size(r, 1) 1]));
X = abs(X);
Y = r.*sin(repmat(th, [size(r, 1) 1]));

forward_coord = floor(linspace(1, x_res, sim_granularity));

xidcs = ceil(Y./costmap_res);
xidcs(ceil(w_granularity/2), :) = zeros(1, sim_granularity);
xidcs = xidcs + floor(x_res/2);
yidcs = ceil(X./costmap_res);
yidcs(:,1) = 1;
yidcs(ceil(w_granularity/2), :) = forward_coord;

idcs = [reshape(xidcs, sim_granularity*w_granularity, 1), reshape(yidcs, sim_granularity*w_granularity, 1)];

idcs = repmat((idcs(:,1) > 0), [1 2]).*idcs;
idcs = repmat((idcs(:,1) < x_res), [1 2]).*idcs;
idcs = repmat((idcs(:,2) < y_res), [1 2]).*idcs;

idcs = idcs + (idcs == 0);
linidcs = sub2ind(size(costmap), idcs(:,1), idcs(:,2));

% costmap(linidcs) = 1;
% imagesc(costmap);
obstacle_cost = costmap(linidcs);
obstacle_cost = reshape(obstacle_cost, size(r));

% may change to sum instead of max
obstacle_cost = max(obstacle_cost, [], 2);

obstacle_cost = 10000*(obstacle_cost == 1) + obstacle_cost;
w_cost = abs(w_space');

obstacle_cost = obstacle_gain * obstacle_cost;
w_cost = w_gain * w_cost;

total_cost = obstacle_cost + w_cost;

[opt_cost, opt_idx] = min(total_cost);

opt_w = w_space(opt_idx);

end

