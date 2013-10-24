function controller()

c = rl_init('controller');
sub = rl_subscribe('pose');

close all;

encoderPhidgetAPI('c'); %connect to encoder
con = maestro(); 

% robot = iRobotCreate(10);
% 
% robot.setworkspace([-1 10 -1.5 1.5]);
% robot.setworldframe(0);
% robot.moveroomba( [ 0, (randi([-3,3],1)), 0] );

% plot([0 43],[.5,.5],'linewidth',3);
% plot([0 43],[-.5,-.5],'linewidth',3);
% plot([0 0],[-.5,.5],'linewidth',3);
% plot([43 43],[-.5,.5],'linewidth',3);


%may need to be 34 (cm)
len = .3;

v_goal = 1;
servo_out = setSpeed(v_goal);

v = 0;
x_pos = 0;
y_pos = 0;
theta = 0;

kp = .0101;
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
%    disp(pose);
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

%    robot.setvel(v,w);    

    alpha = lookup(alpha);
    con.setpos(0, alpha); 
    con.setpos(4, servo_out); 
end

% robot.setvel(0,0);
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
