function [R, t] = get_transformation(ps, pv)
	
	% ps = dlmread('sensor_points');
	% pv = dlmread('vehicle_points');
	
	[m, n] = size(ps);

    pv = pv';
	b = reshape(pv, m*n, 1);
	
	A = [];
	for i=1:m
		A = [A; ps(i,:), 0, 0, 0, 0, 0, 0, 1, 0, 0];
		A = [A; 0, 0, 0, ps(i,:), 0, 0, 0, 0, 1, 0]; 
		A = [A; 0, 0, 0, 0, 0, 0, ps(i,:), 0, 0, 1]; 
	end

    % A
	
	r = A\b;
	R = [r(1:3), r(4:6), r(7:9)];	
	t = r(10:12);
	
	[U, S, V] = svd(R); 
	R = U*eye(size(U,2))*V';
	dlmwrite('rotation', R, '\t');
	dlmwrite('translation', t, '\t');
