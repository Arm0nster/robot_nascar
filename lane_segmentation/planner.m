function planner()

c = rl_init('planner');
sub = rl_subscribe('costmap');
pub = rl_publish('control');

figure;
h = imagesc(zeros(1));
while 1
    rl_spin(10);
    msg = sub.getLatestMessage();
    if isempty(msg)
        continue;
    end
    costmap = msg.data;
    
    control = findBestTrajectory(costmap);

    set(h, 'CDATA', costmap); axis image;
end

end

function [v, w] = findBestTrajectory(costmap)
costmap_res = 0.05;

sim_granularity = 5;
v_max = 5;
w_max = 2;

v_step = .2; 
w_step = .1; 

v_space = 0:v_step:v_max;
w_space = -w_max:w_step:w_max;

rad = conv2(v_space, (1./w_space)');
th = linspace(0, pi/4, sim_granularity);


keyboard;
end
