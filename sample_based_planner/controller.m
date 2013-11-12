function controller()

close all;
c = rl_init('controller');
sub = rl_subscribe('control');

global kbhit;
kbhit = false;
figure('KeyPressFcn', @my_kbhit);

encoderPhidgetAPI('c'); 
con = maestro(); 


drive_servo = 3;
steer_servo = 0;
len = .3;
v_goal = 1;
servo_out = 210; 

v = 0;
x_pos = 0;

con.setaccel(drive_servo, 10);
con.setspeed(drive_servo, 254);

while x_pos < 200 && ~kbhit 

    rl_spin(40);
    msg = sub.getLatestMessage();
    if isempty(msg)
        continue;
    end
    control = msg.data;

    encoderPhidgetAPI('r');
    
    [dx, t] = posUpdate();
    x_pos = x_pos + dx;

    if t ~= 0
        v = dx/(t/1000);
    else
        v = 0;
    end

    w = control;
    alpha = atan(len*w/v)*180/pi;

    if abs(w) > 1000
        alpha = 0;
    end


    alpha = lookup(alpha);

    con.setpos(steer_servo, alpha); 
    con.setpos(drive_servo, servo_out); 
end

con.reset(steer_servo);
con.reset(drive_servo);

encoderPhidgetAPI('d');

end

function [x, t] = posUpdate()
	
	[pos, t] = encoderPhidgetAPI('state_update');
	circ = 0.05*2*pi;
	ratio = -11/29*3.36; %encoder to wheel
	x = pos/4096*circ*ratio; %distance since last update
        
end

function pos = lookup(angle)
pos = angle*-4.598 + 113.952;
if pos > 254
	pos = 254;
elseif pos < 0
	pos = 0;
end
end



