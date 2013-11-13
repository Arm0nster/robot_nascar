function planner()
close all;

c = rl_init('planner');
sub = rl_subscribe('costmap');
pub = rl_publish('control');

global costmap_res;
global costmap_x_res;
global costmap_y_res;
global w_gain;
global obstacle_gain;
global v_max;
global w_min;
global w_max;
global w_granularity;

costmap_res = 0.05;
costmap_x_res = 3*(1/costmap_res);
costmap_y_res = 3*(1/costmap_res);
w_gain = 35;
obstacle_gain = 100;
v_max = 3.5;
w_min = -.5;
w_max = .5;
w_granularity = 15; 

w_space = getSampleSpace();
[idcs, space_dim] = getIndices(w_space);

figure;
h = imagesc(zeros(1));
while 1
    rl_spin(10);
    msg = sub.getLatestMessage();
    if isempty(msg)
        continue;
    end
    costmap = msg.data;

    [control, costmap] = findBestTrajectory(costmap, w_space, idcs, space_dim);

    msg = Message('control', control);
    pub.publish(msg);

    disp(control);
    

    costmap = flipdim(costmap, 1);
    set(h, 'CDATA', costmap); axis image;
end

end

function [opt_w, costmap] = findBestTrajectory(costmap, w_space, idcs, space_dim)
global w_gain;
global obstacle_gain;


obstacle_cost = costmap(idcs);
obstacle_cost = reshape(obstacle_cost, space_dim);

obstacle_cost = sum(obstacle_cost, 1);
% obstacle_cost = max(obstacle_cost, [], 1);

obstacle_cost = 10000*(obstacle_cost == 1) + obstacle_cost;
w_cost = abs(w_space);

obstacle_cost = obstacle_gain * obstacle_cost;
w_cost = w_gain * w_cost;

total_cost = obstacle_cost + w_cost;


[opt_cost, opt_idx] = min(total_cost);
opt_w = w_space(opt_idx);

t = reshape(idcs, space_dim);
costmap(t(:, opt_idx)) = 1;

end

function [idcs, space_dim] = getIndices(w_space)
global costmap_res;
global costmap_x_res;
global costmap_y_res;
global w_granularity;
global v_max;

rads = v_max./w_space';

rads = rads(1:ceil(w_granularity/2));

R = abs(rads./costmap_res);
traj_points = arrayfun(@midPointCircle, R, 'UniformOutput', false);
traj_points = arrayfun(@filterPoints, traj_points, 'UniformOutput', false);

space_dim = [size(traj_points{1}, 1) w_granularity];

straight_traj = traj_points{end};
traj_points(end) = [];
idcs = cell2mat(traj_points);
mirror = [-1.*idcs(:,1), idcs(:,2)];
idcs = [mirror; straight_traj; idcs];
idcs(:,1) = idcs(:,1) + floor(costmap_x_res/2);

idcs = sub2ind([costmap_x_res costmap_y_res], idcs(:,1), idcs(:,2));

end

function [idcs] = filterPoints(traj_points)
global costmap_x_res;
global costmap_y_res;

traj_points = cell2mat(traj_points);
traj_points = flipdim(traj_points, 2);

x_filter = int16((traj_points(:,1) < costmap_x_res) & (traj_points(:,1) > -1));
y_filter = int16((traj_points(:,2) < costmap_y_res) & (traj_points(:,2) > 1));

traj_points = traj_points.*[x_filter x_filter];
traj_points = traj_points.*[y_filter y_filter];
traj_points = setdiff(traj_points, [0 0], 'rows');

idcs = traj_points;

end

function [space] = getSampleSpace()
global w_min;
global w_max;
global w_granularity;
space = linspace(w_min, w_max, w_granularity);
end

function [idcs] = midPointCircle(radius)
xc = 0;
yc = int16(radius);

x = int16(0);
y = int16(radius);
d = int16(1 - radius);

idcs = [
xc, yc+y;
xc, yc-y;
xc+y, yc;
xc-y, yc;
];

while (x < y - 1)
    x = x + 1;
    if (d < 0)
        d = d + x + x + 1;
    else
        y = y - 1;
        a = x -y + 1;
        d = d + a + a;
    end
    idcs = [
    idcs;
     x+xc,  y+yc;
     y+xc,  x+yc;
     y+xc, -x+yc;
     x+xc, -y+yc;
    -x+xc, -y+yc;
    -y+xc, -x+yc;
    -y+xc,  x+yc;
    -x+xc,  y+yc;
    ];
end

end
