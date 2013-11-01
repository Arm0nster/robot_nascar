function figure_sample(evt)
	global space_hit;
	global return_hit;
    global servo_port;
    global motor_port;
	global m; %connection to maestro

	if strcmp(evt.Key, 'space')
		space_hit = true;
	elseif strcmp(evt.Key, 'return')
		return_hit = true;
	end


	if strcmp(evt.Key, 'k')
		disp('Stopped...');
		m.reset(servo_port);
		m.reset(motor_port);
	elseif strcmp(evt.Key, 'w') %speed up
		if m.getpos(motor_port) < 190
			m.setpos(motor_port, m.getpos(motor_port) + 1);
			disp(sprintf('Speed: %d\n', m.getpos(motor_port)));
		end
	elseif strcmp(evt.Key, 's') %speed down
		if m.getpos(motor_port) > 100
			m.setpos(motor_port, m.getpos(motor_port) - 1);
			disp(sprintf('Speed: %d\n', m.getpos(motor_port)));
		end
	elseif strcmp(evt.Key, 'a') %steer left hard
		if m.getpos(servo_port) > 10
			m.setpos(servo_port, m.getpos(servo_port) - 10);
			disp(sprintf('Angle: %d\n', m.getpos(servo_port)));
		end
	elseif strcmp(evt.Key, 'd') %steer right hard
		if m.getpos(servo_port) < 234
			m.setpos(servo_port, m.getpos(servo_port) + 10);
			disp(sprintf('Angle: %d\n', m.getpos(servo_port)));
		end
	elseif strcmp(evt.Key, 'q') %steer left small
		if m.getpos(servo_port) > 0
			m.setpos(servo_port, m.getpos(servo_port) - 1);
			disp(sprintf('Angle: %d\n', m.getpos(servo_port)));
		end
	elseif strcmp(evt.Key, 'e') %steer right small
		if m.getpos(0) < 254
			m.setpos(servo_port, m.getpos(servo_port) + 1);
			disp(sprintf('Angle: %d\n', m.getpos(servo_port)));
		end
	end
	

end

