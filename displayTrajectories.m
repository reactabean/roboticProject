outputFile = csvread('../currentPosition.csv', 1, 0);

initialPos = outputFile(1, :);
intermediatePos = outputFile(2:5);

findBreaks = find(~outputFile(:, 3));
theta1values = outputFile(findBreaks(1)+1:findBreaks(2)-1, 1);
theta2values = outputFile(findBreaks(1)+1:findBreaks(2)-1, 2);
distance3values = outputFile(findBreaks(1)+1:findBreaks(2)-1, 3);
theta4values = outputFile(findBreaks(1)+1:findBreaks(2)-1, 4);


for i = 2:3
    theta1values = [theta1values; outputFile(findBreaks(i)+1:findBreaks(i+1)-1, 1)];
    theta2values = [theta2values; outputFile(findBreaks(i)+1:findBreaks(i+1)-1, 2)];
    distance3values = [distance3values; outputFile(findBreaks(i)+1:findBreaks(i+1)-1, 3)];
    theta4values = [theta4values; outputFile(findBreaks(i)+1:findBreaks(i+1)-1, 4)];
end
theta1values = [theta1values; outputFile(findBreaks(4)+1:end, 1)];
theta2values = [theta2values; outputFile(findBreaks(4)+1:end, 2)];
distance3values = [distance3values; outputFile(findBreaks(4)+1:end, 3)];
theta4values = [theta4values; outputFile(findBreaks(4)+1:end, 4)];

f1 = figure;
p1 = subplot(2, 2, 1);
plot(theta1values, 'LineWidth', 2);
axis(p1, [0 size(theta1values, 1) -180 180]);
vec = ones(size(theta1values, 1), 2);
vec(:, 1) = 150;
vec(:, 2) = -150;
hold on;
plot(vec, 'color', 'red');
title('Values for \theta 1');

if (theta1values(end) > 0)
    legend('Joint Position', 'Joint Limits', 'Location', 'southeast');
else
    legend('Joint Position', 'Joint Limits', 'Location', 'northeast');
end

xlabel('Time (ms)');
ylabel('Joint Position (\theta)');
plot(findBreaks(2)-findBreaks(1),theta1values(findBreaks(2)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')
plot(findBreaks(3)-findBreaks(1),theta1values(findBreaks(3)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')
plot(findBreaks(4)-findBreaks(1),theta1values(findBreaks(4)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')
plot(size(theta1values, 1)-findBreaks(1),theta1values(size(theta1values, 1)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')


p2 = subplot(2, 2, 2);
plot(theta2values, 'LineWidth', 2);
axis(p2, [0 size(theta2values, 1) -180 180]);
vec = ones(size(theta2values, 1), 2);
vec(:, 1) = 100;
vec(:, 2) = -100;
hold on;
plot(vec, 'color', 'red');
title('Values for \theta 2');

if (theta2values(end) > 0)
    legend('Joint Position', 'Joint Limits', 'Location', 'southeast');
else
    legend('Joint Position', 'Joint Limits', 'Location', 'northeast');
end
xlabel('Time (ms)');
ylabel('Joint Position (\theta)');
plot(findBreaks(2)-findBreaks(1),theta2values(findBreaks(2)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')
plot(findBreaks(3)-findBreaks(1),theta2values(findBreaks(3)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')
plot(findBreaks(4)-findBreaks(1),theta2values(findBreaks(4)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')
plot(size(theta2values, 1)-findBreaks(1),theta2values(size(theta2values, 1)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')

p3 = subplot(2, 2, 3);
plot(distance3values, 'LineWidth', 2);
axis(p3, [0 size(distance3values, 1) -210 -90]);
vec = ones(size(distance3values, 1), 2);
vec(:, 1) = -100;
vec(:, 2) = -200;
hold on;
plot(vec, 'color', 'red');
title('Values for distance 3');

if (distance3values(end) > -150)
    legend('Joint Position', 'Joint Limits', 'Location', 'southeast');
else
    legend('Joint Position', 'Joint Limits', 'Location', 'northeast');
end
xlabel('Time (ms)');
ylabel('Joint Position (mm)');
plot(findBreaks(2)-findBreaks(1),distance3values(findBreaks(2)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')
plot(findBreaks(3)-findBreaks(1),distance3values(findBreaks(3)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')
plot(findBreaks(4)-findBreaks(1),distance3values(findBreaks(4)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')
plot(size(distance3values, 1)-findBreaks(1),distance3values(size(distance3values, 1)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')

p4 = subplot(2, 2, 4);
plot(theta4values, 'LineWidth', 2);
axis(p4, [0 size(theta4values, 1) -180 180]);
vec = ones(size(theta4values, 1), 2);
vec(:, 1) = 160;
vec(:, 2) = -160;
hold on;
plot(vec, 'color', 'red');
title('Values for \theta 4');

if (theta4values(end) > 0)
    legend('Joint Position', 'Joint Limits', 'Location', 'southeast');
else
    legend('Joint Position', 'Joint Limits', 'Location', 'northeast');
end
xlabel('Time (ms)');
ylabel('Joint Position (\theta)');
plot(findBreaks(2)-findBreaks(1),theta4values(findBreaks(2)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')
plot(findBreaks(3)-findBreaks(1),theta4values(findBreaks(3)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')
plot(findBreaks(4)-findBreaks(1),theta4values(findBreaks(4)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')
plot(size(theta4values, 1)-findBreaks(1),theta4values(size(theta4values, 1)-findBreaks(1)),'r.','MarkerSize',15, 'color', 'blue')

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