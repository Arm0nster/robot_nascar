function controller()

c = rl_init('controller');
sub = rl_subscribe('pose');

close all;

encoderPhidgetAPI('c'); 
con = maestro(); 

len = .3;

v_goal = 1;
servo_out = setSpeed(v_goal);

v = 0;
x_pos = 0;
y_pos = 0;
theta = 0;

kp = .101;
kd = 2*sqrt(kp);

con.setaccel(4, 10);
con.setspeed(4, 254);

while 1 

    rl_spin(10);
    msg = sub.getLatestMessage();
    if isempty(msg)
        continue;
    end
    pose = msg.data;
    y_pos = pose(1);
    theta = pose(2);

    encoderPhidgetAPI('r');
    
    [dx, t] = posUpdate();
    x_pos = x_pos + dx;

    if t ~= 0
        v = dx/(t/100);
    else
        v = 0;
    end

    w = (-kd*tan(theta)) + (-(kp*(y_pos))/(v*cos(theta)));
    alpha = atan(len*w/v)*180/pi;

    if abs(w) > 1000
        % for simulator add maximum
        % w = 5;
        alpha = 0;
    end


    alpha = lookup(alpha);
    con.setpos(0, alpha); 
    con.setpos(4, servo_out); 
end

con.reset(0);
con.reset(4);

encoderPhidgetAPI('d');

end

function [x, t] = posUpdate()
	
	[pos, t] = encoderPhidgetAPI('state_update');
	circ = 0.05*2*pi;
	ratio = -11/29; %encoder to wheel
	x = pos/4096*circ*ratio; %distance since last update
        
end

%-51.41 to 42.69
function pos = lookup(angle)
angle = angle + 90;
pos = round((angle - 132.69)/(-0.32));
if pos > 254
	pos = 254;
elseif pos < 0
	pos = 0;
end
end

function p = setSpeed(velocity)
if (velocity == 10)
    p = 254;
else
    if(velocity == 1)
        p = 200;
    end
end
end
