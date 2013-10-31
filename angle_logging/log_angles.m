function log_angles(s_port, m_port)

	global space_hit;
	global return_hit;
	global servo_port;
	global motor_port;
	global m;

	%connections
	m = maestro();
	encoderPhidgetAPI('c');

	m.setpos(s_port, 127);
	m.setpos(m_port, 127);

	space_hit = false;
	return_hit = false;
	servo_port = s_port
	motor_port = m_port

	angles = fopen('angles.dat', 'w+');

	figure('KeyPressFcn', @(obj, evt)figure_sample(evt));

	encoderPhidgetAPI('r');

	LENGTH = 0.34;
	velocities = []; %plots speed over time
	max_points = 20;
	points = 0;
	

	%press space to begin tracking car position, enter to quit
	%fprintf(angles, 'Header: delta(degrees) servo_pos\n');
	while ~return_hit

		%wait for car to equilibriate in circle	
		if space_hit
			space_hit = false;
			
			disp(sprintf('Velocity at start: %.2f\n', ticksToMeters(x)/t));	

			%press space to stop loop
			while ~space_hit
				pause(0.1);
			end
			[x,t] = encoderPhidgetAPI('s');

			circum = ticksToMeters(x);
			radius = circum/(2*pi);

			vel = circum/(t/1000);
			motor_speed = m.getpos(motor_port);
			servo_pos = m.getpos(servo_port);

			delta = atan(LENGTH/radius) * 180/pi;

			if servo_pos > 127
				delta = -delta;
			end
			%fprintf(angles, '%0.2f %d %0.2f %d %d\n', circum, t, vel, motor_speed, servo_pos);
			%need turning angle and servo_pos
			fprintf(angles, '%0.3f %d\n', delta, servo_pos);
			disp(sprintf('%0.3f %d\n', delta, servo_pos));
            		disp(sprintf('Captured\n'));
            		space_hit = false;
		end

		[x,t] = encoderPhidgetAPI('s');
		
		disp(sprintf('Velocity: %0.2f\n', x/(t/1000)));
%		points = points + 1;
%
%		if points <= max_points
%			velocities(points, :) = [t, x/(t/1000)];
%		else
%			velocities = velocities(2:end, :);
%			velocities(end+1, :) = [t, x/(t/1000)];
%		end
%
%		low_t = velocities(1, 1);
%		high_t = velocities(end, 1);
%		
%		h = plot(velocities(:, 1), velocities(:, 2), 'o');
%		n_axis = ([,,,]); 
%
%		set(h, 'XData', velocities(:, 1), 'YData', velocities(:, 2));
%
%		scatter(velocities(:, 1), velocities(:, 2));
%		zoom;
		pause(0.2);
	end



	m.reset(servo_port);
	m.reset(motor_port);
	encoderPhidgetAPI('d');

    close all;

end

function m = ticksToMeters(ticks)

	circ = 0.05*2*pi;
	ratio = -11/29*3.36;
	m = ticks/4096*circ*ratio;

end

%Notes:
% Perform measurements at increments of 5 from 220 to 130
% Repeat for other side.
%
% At 200 servo position, perform the measurements at various
% speeds, perhands 190 to 160 in increments of 5.
%

