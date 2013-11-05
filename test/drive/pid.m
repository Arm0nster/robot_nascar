function pid()
c = rl_init('pid');
pose_sub = rl_subscribe('pose');
speed_sub = rl_subscribe('speed');
pub = rl_publish('control');

len = .3;
theta = 0;
y_pos = 0;
y_diff = 0;

kp = 1.7;
kd = 2*sqrt(kp);
ki = 0;

while 1
    rl_spin(10);
    pose_msg = pose_sub.getLatestMessage();
    if isempty(pose_msg)
        continue;
    end
    pose = pose_msg.data;
    y_pos = pose(1);
    theta = pose(2);

    speed_msg = speed_sub.getLatestMessage();
    if isempty(speed_msg)
        continue;
    end
    v = speed_msg.data;

    w = (-kd*tan(theta)) - ((kp*(y_pos))/(v*cos(theta)));
    alpha = atan(len*w/v)*180/pi;
    if abs(w) > 1000
        alpha = 0;
    end

    msg = Message('control', alpha);
    pub.publish(msg);
end

end
