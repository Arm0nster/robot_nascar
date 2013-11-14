function planner()
close all;

c = rl_init('planner');
sub = rl_subscribe('costmap');
pub = rl_publish('control');

% parameters
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
w_min = -2.5;
w_max = 2.5;
w_granularity = 21; 

w_space = getSampleSpace();
[idcs, space_dim] = getIndices(w_space);

% figure;
% h = imagesc(zeros(1));
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

    % disp(control);
    % costmap(idcs) = 1;
    % costmap = flipdim(costmap, 1);
    % set(h, 'CDATA', costmap); axis image;
end

end

% uses the indices each trajectory would occupy to evaluate the cost of
% following such a trajectory, and then returns the one of the lowest cost
function [opt_w, costmap] = findBestTrajectory(costmap, w_space, idcs, space_dim)
global w_gain;
global obstacle_gain;

obstacle_cost = costmap(idcs);
obstacle_cost = reshape(obstacle_cost, space_dim);

obstacle_cost = sum(obstacle_cost, 1);

obstacle_cost = 10000*(obstacle_cost == 1) + obstacle_cost;
w_cost = abs(w_space);

obstacle_cost = obstacle_gain * obstacle_cost;
w_cost = w_gain * w_cost;

total_cost = obstacle_cost + w_cost;


[opt_cost, opt_idx] = min(total_cost);
opt_w = w_space(opt_idx);

% Uncomment to display optimal trajectory
% t = reshape(idcs, space_dim);
% costmap(t(:, opt_idx)) = 1;

end

% returns the indices that the set of possible trajectories would traverse
% in our costmap
function [idcs, space_dim] = getIndices(w_space)
global costmap_res;
global costmap_x_res;
global costmap_y_res;
global w_granularity;
global v_max;

rads = v_max./w_space';
R = rads./costmap_res;

R_pos = R(find(R > 0));
R_neg = R(find(R < 0));

tp_pos = arrayfun(@midPointCircle, R_pos, R_pos, 'UniformOutput', false);
tp_neg = arrayfun(@midPointCircle, abs(R_neg), R_neg, 'UniformOutput', false);
traj_points = [tp_neg; tp_pos];

traj_points = arrayfun(@flipxy, traj_points, 'UniformOutput', false);
traj_points = arrayfun(@filterPoints, traj_points, 'UniformOutput', false);

set_sizes = cellfun(@size, traj_points, 'UniformOutput', false);
set_sizes = cell2mat(set_sizes);
set_sizes = set_sizes(:,1);
min_size = min(set_sizes);
for i=1:size(traj_points,1)
    t = traj_points{i};
    t = sortrows(t, 2);
    traj_points{i} = t(1:min_size, :);
end

space_dim = [min_size w_granularity];

idcs = cell2mat(traj_points);
idcs = sub2ind([costmap_x_res costmap_y_res], idcs(:,1), idcs(:,2));

end

% crops the indices of our circles to only those points that would fit in the costmap
function [idcs] = filterPoints(traj_points)
global costmap_x_res;
global costmap_y_res;

traj_points = cell2mat(traj_points);
traj_points(:,1) = traj_points(:,1) + floor(costmap_x_res/2);

x_filter = int16((traj_points(:,1) < costmap_x_res) & (traj_points(:,1) > 1));
y_filter = int16((traj_points(:,2) < costmap_y_res) & (traj_points(:,2) > 1));

traj_points = traj_points.*[x_filter x_filter];
traj_points = traj_points.*[y_filter y_filter];
traj_points = setdiff(traj_points, [0 0], 'rows');

idcs = traj_points;

end

% creates our sample space of possible omega values
function [space] = getSampleSpace()
global w_min;
global w_max;
global w_granularity;
space = linspace(w_min, w_max, w_granularity);
end

% finds the indices a circle would occupy in matrix using bresenhams circle algorithm
function [idcs] = midPointCircle(radius, yc)
xc = 0;
yc = int16(yc);

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

function [idcs] = flipxy(traj_points)
traj_points = cell2mat(traj_points);
traj_points = flipdim(traj_points, 2);
idcs = traj_points;
end
