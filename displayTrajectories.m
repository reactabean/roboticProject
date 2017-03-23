outputFile = csvread('../currentPosition.csv', 1, 0);

initialPos = outputFile(1, 1:4);
intermediatePos = outputFile(2:5, 1:4);
theta1spline = outputFile(6:4:18, 1:4);
theta2spline = outputFile(7:4:19, 1:4);
distance3spline = outputFile(8:4:20, 1:4);
theta4spline = outputFile(9:4:21, 1:4);

findBreaks = find(~outputFile(:, 3) .* ~outputFile(:, 1));
theta1values = outputFile(findBreaks(1)+1:findBreaks(2)-1, 1);
theta2values = outputFile(findBreaks(1)+1:findBreaks(2)-1, 2);
distance3values = outputFile(findBreaks(1)+1:findBreaks(2)-1, 3);
theta4values = outputFile(findBreaks(1)+1:findBreaks(2)-1, 4);
timeValues = outputFile(findBreaks(1)+1:findBreaks(2)-1, 5);


for i = 2:3
    theta1values = [theta1values; outputFile(findBreaks(i)+1:findBreaks(i+1)-1, 1)];
    theta2values = [theta2values; outputFile(findBreaks(i)+1:findBreaks(i+1)-1, 2)];
    distance3values = [distance3values; outputFile(findBreaks(i)+1:findBreaks(i+1)-1, 3)];
    theta4values = [theta4values; outputFile(findBreaks(i)+1:findBreaks(i+1)-1, 4)];
    timeValues = [timeValues; outputFile(findBreaks(i)+1:findBreaks(i+1)-1, 5)];
end
theta1values = [theta1values; outputFile(findBreaks(4)+1:end, 1)];
theta2values = [theta2values; outputFile(findBreaks(4)+1:end, 2)];
distance3values = [distance3values; outputFile(findBreaks(4)+1:end, 3)];
theta4values = [theta4values; outputFile(findBreaks(4)+1:end, 4)];
timeValues = [timeValues; outputFile(findBreaks(4)+1:end, 5)];

% hack
while (mod(size(theta1values, 1), 4) > 0)
    theta1values = [theta1values; outputFile(end, 1)];
    theta2values = [theta2values; outputFile(end, 2)];
    distance3values = [distance3values; outputFile(end, 3)];
    theta4values = [theta4values; outputFile(end, 4)];
    timeValues = [timeValues; outputFile(end, 5)];
end

f1 = figure;
p1 = subplot(2, 2, 1);
plot(timeValues, theta1values, 'LineWidth', 2);
axis(p1, [0 timeValues(end) -180 180]);
vec = ones(size(theta1values, 1), 2);
vec(:, 1) = 150;
vec(:, 2) = -150;
hold on;
plot(timeValues, calcSplineValues(theta1spline, size(theta1values, 1), 4), 'color', 'green');
plot(timeValues, vec, 'color', 'red');
title('Values for \theta 1');

if (theta1values(end) > 0)
    legend('Joint Position', 'Ideal Path', 'Joint Limits', 'Location', 'southeast');
else
    legend('Joint Position', 'Ideal Path', 'Joint Limits', 'Location', 'northeast');
end

xlabel('Time (ms)');
ylabel('Joint Position (\theta)');
plot(timeValues(findBreaks(2)-findBreaks(1)),theta1values(findBreaks(2)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')
plot(timeValues(findBreaks(3)-findBreaks(1)),theta1values(findBreaks(3)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')
plot(timeValues(findBreaks(4)-findBreaks(1)),theta1values(findBreaks(4)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')
plot(timeValues(end),theta1values(size(theta1values, 1)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')

plot(0,theta1spline(1, 1),'r.','MarkerSize',15, 'color', 'green')
plot(timeValues(size(theta1values, 1)/4),theta1spline(2, 1),'r.','MarkerSize',15, 'color', 'green')
plot(timeValues(size(theta1values, 1)*2/4),theta1spline(3, 1),'r.','MarkerSize',15, 'color', 'green')
plot(timeValues(size(theta1values, 1)*3/4),theta1spline(4, 1),'r.','MarkerSize',15, 'color', 'green')
plot(timeValues(size(theta1values, 1)),sum(theta1spline(4, :)),'r.','MarkerSize',15, 'color', 'green')

p2 = subplot(2, 2, 2);
plot(timeValues, theta2values, 'LineWidth', 2);
axis(p2, [0 timeValues(end) -180 180]);
vec = ones(size(theta2values, 1), 2);
vec(:, 1) = 100;
vec(:, 2) = -100;
hold on;
plot(timeValues, calcSplineValues(theta2spline, size(theta1values, 1), 4), 'color', 'green');
plot(timeValues, vec, 'color', 'red');
title('Values for \theta 2');

if (theta2values(end) > 0)
    legend('Joint Position', 'Ideal Path', 'Joint Limits', 'Location', 'southeast');
else
    legend('Joint Position', 'Ideal Path', 'Joint Limits', 'Location', 'northeast');
end
xlabel('Time (ms)');
ylabel('Joint Position (\theta)');
plot(timeValues(findBreaks(2)-findBreaks(1)),theta2values(findBreaks(2)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')
plot(timeValues(findBreaks(3)-findBreaks(1)),theta2values(findBreaks(3)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')
plot(timeValues(findBreaks(4)-findBreaks(1)),theta2values(findBreaks(4)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')
plot(timeValues(end),theta2values(size(theta2values, 1)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')

plot(0,theta2spline(1, 1),'r.','MarkerSize',15, 'color', 'green')
plot(timeValues(size(theta2values, 1)/4),theta2spline(2, 1),'r.','MarkerSize',15, 'color', 'green')
plot(timeValues(size(theta2values, 1)*2/4),theta2spline(3, 1),'r.','MarkerSize',15, 'color', 'green')
plot(timeValues(size(theta2values, 1)*3/4),theta2spline(4, 1),'r.','MarkerSize',15, 'color', 'green')
plot(timeValues(size(theta2values, 1)),sum(theta2spline(4, :)),'r.','MarkerSize',15, 'color', 'green')

p3 = subplot(2, 2, 3);
plot(timeValues, distance3values, 'LineWidth', 2);
axis(p3, [0 timeValues(end) -210 -90]);
vec = ones(size(distance3values, 1), 2);
vec(:, 1) = -100;
vec(:, 2) = -200;
hold on;
plot(timeValues, calcSplineValues(distance3spline, size(theta1values, 1), 4), 'color', 'green');
plot(timeValues, vec, 'color', 'red');
title('Values for distance 3');

if (distance3values(end) > -150)
    legend('Joint Position', 'Ideal Path', 'Joint Limits', 'Location', 'southeast');
else
    legend('Joint Position', 'Ideal Path', 'Joint Limits', 'Location', 'northeast');
end
xlabel('Time (ms)');
ylabel('Joint Position (mm)');
plot(timeValues(findBreaks(2)-findBreaks(1)),distance3values(findBreaks(2)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')
plot(timeValues(findBreaks(3)-findBreaks(1)),distance3values(findBreaks(3)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')
plot(timeValues(findBreaks(4)-findBreaks(1)),distance3values(findBreaks(4)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')
plot(timeValues(end),distance3values(size(distance3values, 1)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')

plot(0,distance3spline(1, 1),'r.','MarkerSize',15, 'color', 'green')
plot(timeValues(size(theta1values, 1)/4),distance3spline(2, 1),'r.','MarkerSize',15, 'color', 'green')
plot(timeValues(size(theta1values, 1)*2/4),distance3spline(3, 1),'r.','MarkerSize',15, 'color', 'green')
plot(timeValues(size(theta1values, 1)*3/4),distance3spline(4, 1),'r.','MarkerSize',15, 'color', 'green')
plot(timeValues(size(theta1values, 1)),sum(distance3spline(4, :)),'r.','MarkerSize',15, 'color', 'green')

p4 = subplot(2, 2, 4);
plot(timeValues, theta4values, 'LineWidth', 2);
axis(p4, [0 timeValues(end) -180 180]);
vec = ones(size(theta4values, 1), 2);
vec(:, 1) = 160;
vec(:, 2) = -160;
hold on;
plot(timeValues, calcSplineValues(theta4spline, size(theta1values, 1), 4), 'color', 'green');
plot(timeValues, vec, 'color', 'red');
title('Values for \theta 4');

if (theta4values(end) > 0)
    legend('Joint Position', 'Ideal Path', 'Joint Limits', 'Location', 'southeast');
else
    legend('Joint Position', 'Ideal Path', 'Joint Limits', 'Location', 'northeast');
end
xlabel('Time (ms)');
ylabel('Joint Position (\theta)');
plot(timeValues(findBreaks(2)-findBreaks(1)),theta4values(findBreaks(2)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')
plot(timeValues(findBreaks(3)-findBreaks(1)),theta4values(findBreaks(3)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')
plot(timeValues(findBreaks(4)-findBreaks(1)),theta4values(findBreaks(4)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')
plot(timeValues(end),theta4values(size(theta4values, 1)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')

plot(0,theta4spline(1, 1),'r.','MarkerSize',15, 'color', 'green')
plot(timeValues(size(theta4values, 1)/4),theta4spline(2, 1),'r.','MarkerSize',15, 'color', 'green')
plot(timeValues(size(theta4values, 1)*2/4),theta4spline(3, 1),'r.','MarkerSize',15, 'color', 'green')
plot(timeValues(size(theta4values, 1)*3/4),theta4spline(4, 1),'r.','MarkerSize',15, 'color', 'green')
plot(timeValues(size(theta4values, 1)),sum(theta4spline(4, :)),'r.','MarkerSize',15, 'color', 'green')


set (f1, 'Units', 'normalized', 'Position', [0,0,1,1]);

f2 = figure;
yValues = sind(theta1values)*195 + sind(theta1values+theta2values)*142;
xValues = cosd(theta1values)*195 + cosd(theta1values+theta2values)*142;
p5 = plot(xValues,yValues, 'color', 'black', 'LineWidth', 2);
axis([-337 337 -337 337], 'equal');
hold on;
plot(0,0,'r.','MarkerSize',15, 'color', 'black', 'LineWidth', 20);
plot(xValues(1),yValues(1),'r.','MarkerSize',25, 'color', [.5, 0, .5]);
plot(xValues(findBreaks(2)-findBreaks(1)),yValues(findBreaks(2)-findBreaks(1)),'r.','MarkerSize',25, 'color', 'blue');
plot(xValues(findBreaks(3)-findBreaks(1)),yValues(findBreaks(3)-findBreaks(1)),'r.','MarkerSize',25, 'color', 'green');
plot(xValues(findBreaks(4)-findBreaks(1)),yValues(findBreaks(4)-findBreaks(1)),'r.','MarkerSize',25, 'color', 'yellow');
plot(xValues(end),yValues(end),'r.','MarkerSize',25, 'color', [1 .65 0]);
theta = linspace(-180, 180, size(xValues, 1));
circleX = 337*cosd(theta);
circleY = 337*sind(theta);
plot(circleX, circleY, 'color', 'red');
legend('Joint movement path', 'Origin', 'Point 0', 'Point 1', 'Point 2', 'Point 3', 'Point 4', 'outer joint limit', 'location', 'southeast');
xlabel('X position (mm)');
ylabel('X position (mm)');
title('X, Y position of end effector');
set (f2, 'Units', 'normalized', 'Position', [0,0,1,1]);