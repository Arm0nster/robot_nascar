function fit_data(file)

	out = fopen('steering_fit.dat', 'w+');

	data = load(file);
	p = polyfit(data(:, 1), data(:, 2), 1) %linear regression p(1) slope, p(2) y-intercept

	fprintf(out, '%0.3f\n%0.3f', p(1), p(2));
	axis ([-90 90 0 254]);
	refline(p(1), p(2));
	hold on;
	plot(data(:, 1), data(:, 2), 'o');

end


function [a, b] = refit(data)
x = data(:,1);
y = data(:,2);

C = [x ones(size(x))];
p = C\y;

a = p(1);
b = p(2);
end


